// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Round-trip tests for the idyntree-model-json format.
 *
 * For each random model generated with iDynTree::getRandomModel:
 *  1. Export the model to an idyntree-model-json string.
 *  2. Import the model back.
 *  3. Verify that all attributes of the original and round-tripped model match.
 */

#include "JSONModelRoundTripTestUtils.h"
#include "ModelJSONImportExport.h"
#include "testModels.h"

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/TestUtils.h>

#include <cstdlib>
#include <iostream>
#include <string>

using namespace iDynTree;

// ─── test cases ───────────────────────────────────────────────────────────────

static void testRoundTripWithJointTypes(unsigned int nrOfJoints,
                                        unsigned int allowedJointTypes,
                                        const std::string& label)
{
    std::cout << "Testing round-trip: " << label << " (" << nrOfJoints << " joints)" << std::endl;

    const Model originalModel = getRandomModel(nrOfJoints, 5, allowedJointTypes);

    std::string jsonString;
    ASSERT_IS_TRUE(modelToJSONString(originalModel, jsonString));

    Model loadedModel;
    ASSERT_IS_TRUE(modelFromJSONString(jsonString, loadedModel));

    assertModelsAreEqualIncludingShapesAndSensors(originalModel, loadedModel);
}

static void testVersionMismatchIsRejected()
{
    // Manually craft a JSON string with a wrong version number
    std::string badJSON = R"({"idyntree_model_json_version": 9999, "default_base_link": "base",
        "links": [], "joints": [], "additional_frames": []})";
    Model model;
    // Suppress expected error output
    ASSERT_IS_FALSE(modelFromJSONString(badJSON, model));
}

static void testMissingVersionIsRejected()
{
    std::string badJSON = R"({"default_base_link": "base",
        "links": [], "joints": [], "additional_frames": []})";
    Model model;
    ASSERT_IS_FALSE(modelFromJSONString(badJSON, model));
}

static void testEmptyModelRoundTrip()
{
    // A model with a single link and no joints is the minimal valid model
    Model original;
    Link baseLink;
    SpatialInertia inertia(1.0, Position(0, 0, 0), RotationalInertia());
    inertia.zero();
    // Override with identity inertia for single link
    baseLink.setInertia(inertia);
    original.addLink("base", baseLink);
    original.setDefaultBaseLink(0);

    std::string jsonString;
    ASSERT_IS_TRUE(modelToJSONString(original, jsonString));

    Model loaded;
    ASSERT_IS_TRUE(modelFromJSONString(jsonString, loaded));

    ASSERT_EQUAL_DOUBLE(loaded.getNrOfLinks(), 1);
    ASSERT_EQUAL_DOUBLE(loaded.getNrOfJoints(), 0);
    ASSERT_EQUAL_STRING(loaded.getLinkName(0), "base");
    ASSERT_EQUAL_STRING(loaded.getLinkName(loaded.getDefaultBaseLink()), "base");
}

static void testRoundTripAfterChangingBase()
{
    Model originalModel = getRandomModel(10, 5, ALL_JOINT_TYPES);

    std::string jsonString;
    ASSERT_IS_TRUE(modelToJSONString(originalModel, jsonString));

    Model loadedModel;
    ASSERT_IS_TRUE(modelFromJSONString(jsonString, loadedModel));

    const std::string newBaseLinkName = getRandomLinkOfModel(originalModel);

    const LinkIndex originalNewBaseIdx = originalModel.getLinkIndex(newBaseLinkName);
    const LinkIndex loadedNewBaseIdx = loadedModel.getLinkIndex(newBaseLinkName);

    ASSERT_IS_TRUE(originalNewBaseIdx != LINK_INVALID_INDEX);
    ASSERT_IS_TRUE(loadedNewBaseIdx != LINK_INVALID_INDEX);

    originalModel.setDefaultBaseLink(originalNewBaseIdx);
    loadedModel.setDefaultBaseLink(loadedNewBaseIdx);

    assertModelsAreEqualIncludingShapesAndSensors(originalModel, loadedModel);
}

static void testRoundTripFromURDFFixture(const std::string& urdfFilename)
{
    std::cout << "Testing round-trip fixture: " << urdfFilename << std::endl;
    const std::string urdfPath = getAbsModelPath(urdfFilename);

    ModelLoader loader;
    ASSERT_IS_TRUE(loader.loadModelFromFile(urdfPath, "urdf"));

    const Model originalModel = loader.model();

    std::string jsonString;
    ASSERT_IS_TRUE(modelToJSONString(originalModel, jsonString));

    Model loadedModel;
    ASSERT_IS_TRUE(modelFromJSONString(jsonString, loadedModel));

    assertModelsAreEqualIncludingShapesAndSensors(originalModel, loadedModel);
}

int main()
{
    // ── All joint types including Spherical ───────────────────────────────────
    for (unsigned int nrJoints : {1u, 5u, 10u})
    {
        testRoundTripWithJointTypes(nrJoints, ALL_JOINT_TYPES, "ALL_JOINT_TYPES");
    }

    // ── Edge cases ────────────────────────────────────────────────────────────
    testEmptyModelRoundTrip();
    testRoundTripAfterChangingBase();

    // ── Deterministic fixture-based tests (non-random models) ──────────────
    for (unsigned int modelIndex = 0; modelIndex < IDYNTREE_TESTS_URDFS_NR; ++modelIndex)
    {
        testRoundTripFromURDFFixture(IDYNTREE_TESTS_URDFS[modelIndex]);
    }

    testVersionMismatchIsRejected();
    testMissingVersionIsRejected();

    std::cout << "All idyntree-model-json round-trip tests passed." << std::endl;
    return EXIT_SUCCESS;
}
