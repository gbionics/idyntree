// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "XMLParser.h"

#include "XMLAttribute.h"
#include "XMLDocument.h"
#include "XMLElement.h"

#include <iDynTree/Utils.h>

#include <libxml/SAX2.h>
#include <libxml/xmlschemas.h>

#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stack>
#include <vector>

static std::string stringFromFormattedCString(const char* message, va_list arguments)
{
    va_list tempArguments;
    va_copy(tempArguments, arguments);
    const int length = std::vsnprintf(nullptr, 0, message, tempArguments);
    va_end(tempArguments);
    if (length <= 0)
    {
        return {};
    }

    std::string buffer(static_cast<std::size_t>(length), '\0');
    std::vsnprintf(&buffer[0], buffer.size() + 1, message, arguments);
    return buffer;
}

namespace iDynTree
{

// MARK: - XMLParserPimpl definition

class XMLParser::XMLParserPimpl
{

    xmlSAXHandler m_callbackHandler;

public:
    std::stack<std::shared_ptr<XMLElement>> m_parsedTrace;
    std::function<std::shared_ptr<XMLDocument>(XMLParserState&)> f_documentFactory;

    std::shared_ptr<XMLDocument> m_document;

    std::string m_schemaLocation;
    bool m_performValidation;

    bool m_logParsing;

    bool m_keepInMemory;
    std::vector<std::string> m_packageDirs;

    XMLParserState m_parserState;

public:
    XMLParserPimpl()
    {
        // ???: SAX2 structure initialization is kept in PIMPL constructor
        // All the other initializations are left to the main class

        // xmlSAX2InitDefaultSAXHandler(&m_callbackHandler, 0);
        // xmlSAXVersion(&m_callbackHandler, 2);
        // TODO: check what init does as the program crashes in that case
        // I think it initialize some callbacks that are missing
        memset(&m_callbackHandler, 0, sizeof(xmlSAXHandler));
        m_callbackHandler.initialized = XML_SAX2_MAGIC;

        m_callbackHandler.startElementNs = &parserCallbackStartTag;
        m_callbackHandler.endElementNs = &parserCallbackEndTag;
        m_callbackHandler.startDocument = &parserCallbackStartDocument;
        m_callbackHandler.endDocument = &parserCallbackEndDocument;
        m_callbackHandler.characters = &parserCallbackCharacters;
        m_callbackHandler.error = &parserErrorMessageCallback;
        m_callbackHandler.warning = &parserWarningMessageCallback;
    }

    xmlSAXHandlerPtr callbackHandler()
    {
        return &m_callbackHandler;
    }
    // TODO: should we handle entities (i.e. &amp; &lg; etc).
    //        static xmlEntityPtr
    //        my_getEntity(void *user_data, const xmlChar *name) {
    //            return xmlGetPredefinedEntity(name);
    //        }

private:
    static const std::string stringFromXMLCharPtr(const xmlChar* xmlString);
    static const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>
    attributesFromArray(const xmlChar** attributes, int nb_attributes, int nb_defaulted);

    static void parserCallbackStartDocument(void* context);
    static void parserCallbackEndDocument(void* context);
    static void parserCallbackStartTag(void* context,
                                       const xmlChar* localname,
                                       const xmlChar* prefix,
                                       const xmlChar* URI,
                                       int nb_namespaces,
                                       const xmlChar** namespaces,
                                       int nb_attributes,
                                       int nb_defaulted,
                                       const xmlChar** attributes);
    static void parserCallbackEndTag(void* context,
                                     const xmlChar* localname,
                                     const xmlChar* prefix,
                                     const xmlChar* URI);
    static void parserCallbackCharacters(void* ctx, const xmlChar* ch, int len);

    static void parserErrorMessageCallback(void* ctx, const char* msg, ...);
    static void parserWarningMessageCallback(void* ctx, const char* msg, ...);
};

const std::string XMLParser::XMLParserPimpl::stringFromXMLCharPtr(const xmlChar* xmlString)
{
    return std::string(reinterpret_cast<const char*>(xmlString), xmlStrlen(xmlString));
}

const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>
XMLParser::XMLParserPimpl::attributesFromArray(const xmlChar** attributes,
                                               int nb_attributes,
                                               int nb_defaulted)
{
    // TODO: how to handle the nb_defaulted parameter
    const int fields = 5;
    std::unordered_map<std::string, std::shared_ptr<XMLAttribute>> mappedAttributes;
    // Array of array. The inner array contains: localname/prefix/URI/value/end
    for (int attributeIndex = 0; attributeIndex < nb_attributes; ++attributeIndex)
    {
        std::string name = XMLParser::XMLParserPimpl::stringFromXMLCharPtr(
            attributes[attributeIndex * fields + 0]);
        std::string prefix = XMLParser::XMLParserPimpl::stringFromXMLCharPtr(
            attributes[attributeIndex * fields + 1]);
        std::string uri = XMLParser::XMLParserPimpl::stringFromXMLCharPtr(
            attributes[attributeIndex * fields + 2]);
        const xmlChar* value_start = attributes[attributeIndex * fields + 3];
        const xmlChar* value_end = attributes[attributeIndex * fields + 4];
        size_t len = value_end - value_start;
        std::string value(reinterpret_cast<const char*>(value_start), len);

        mappedAttributes.emplace(name, std::make_shared<XMLAttribute>(name, value, prefix, uri));
    }
    return mappedAttributes;
}

void XMLParser::XMLParserPimpl::parserCallbackStartDocument(void* context)
{
    XMLParser* state = static_cast<XMLParser*>(context);
    if (state->m_pimpl->m_logParsing)
    {
        reportInfo("XMLParser", "parserCallbackStartDocument", "Start document");
    }
    // clear stack
    state->m_pimpl->m_parsedTrace = std::stack<std::shared_ptr<XMLElement>>();
    // create a Document type
    state->m_pimpl->m_document = std::shared_ptr<XMLDocument>(
        state->m_pimpl->f_documentFactory(state->m_pimpl->m_parserState));
}

void XMLParser::XMLParserPimpl::parserCallbackEndDocument(void* context)
{
    XMLParser* state = static_cast<XMLParser*>(context);
    if (state->m_pimpl->m_logParsing)
    {
        reportInfo("XMLParser", "parserCallbackEndDocument", "End document");
    }

    if (!state->m_pimpl->m_document->documentHasBeenParsed())
    {
        state->m_pimpl->m_document.reset();
        state->m_pimpl->m_parserState.setParsingError();
        reportError("XMLParser",
                    "parserCallbackEndDocument",
                    "Document final callback failed processing");
    }

    if (!state->m_pimpl->m_parsedTrace.empty())
    {
        // This is an error. Reset the document as it is invalid
        state->m_pimpl->m_document.reset();
        state->m_pimpl->m_parserState.setParsingError();
        reportError("XMLParser", "parserCallbackEndDocument", "Unbalanced tags in the document");
    }
}
void XMLParser::XMLParserPimpl::parserCallbackStartTag(void* context,
                                                       const xmlChar* localname,
                                                       const xmlChar* prefix,
                                                       const xmlChar* URI,
                                                       int nb_namespaces,
                                                       const xmlChar** namespaces,
                                                       int nb_attributes,
                                                       int nb_defaulted,
                                                       const xmlChar** attributes)
{
    // ask to the top of the stack if it can push this element
    // TODO: handle namespaces and prefix
    XMLParser* state = static_cast<XMLParser*>(context);
    const std::string localNameString = XMLParser::XMLParserPimpl::stringFromXMLCharPtr(localname);
    if (state->m_pimpl->m_logParsing)
    {
        // TODO: reportXXX should either accept a format + var arguments or something else
        std::string message = std::string("Start of tag <") + localNameString + (">");
        reportInfo("XMLParser", "parserCallbackStartTag", message.c_str());
    }

    // get attributes
    std::unordered_map<std::string, std::shared_ptr<XMLAttribute>> parsedAttributes
        = XMLParser::XMLParserPimpl::attributesFromArray(attributes, nb_attributes, nb_defaulted);

    if (state->m_pimpl->m_logParsing)
    {
        for (auto pair : parsedAttributes)
        {
            // TODO: reportXXX should either accept a format + var arguments or something else
            std::string message = std::string("Attribute found: ") + pair.second->description();
            reportInfo("XMLParser", "parserCallbackStartTag", message.c_str());
        }
    }

    std::shared_ptr<XMLElement> nextElement;
    // The start tag can be the root or not. If root, we have to ask the children to the
    // Document object
    if (state->m_pimpl->m_parsedTrace.empty())
    {
        // it is the root
        nextElement = state->m_pimpl->m_document->rootElementForName(localNameString,
                                                                     state->m_pimpl->m_packageDirs);
        if (state->m_pimpl->m_keepInMemory)
        {
            state->m_pimpl->m_document->setRootElement(nextElement);
        }
    } else
    {
        nextElement = state->m_pimpl->m_parsedTrace.top()->childElementForName(localNameString);
        if (state->m_pimpl->m_keepInMemory)
        {
            state->m_pimpl->m_parsedTrace.top()->addChildElement(nextElement);
        }
    }

    // additionally configure element
    if (!nextElement->setAttributes(parsedAttributes))
    {
        state->m_pimpl->m_parserState.setParsingError();
    }
    state->m_pimpl->m_parsedTrace.push(nextElement);
}

void XMLParser::XMLParserPimpl::parserCallbackEndTag(void* context,
                                                     const xmlChar* localname,
                                                     const xmlChar* prefix,
                                                     const xmlChar* URI)
{
    // TODO: use the prefix and uri, or remove them from the parameters
    XMLParser* state = static_cast<XMLParser*>(context);
    std::shared_ptr<XMLElement> element = state->m_pimpl->m_parsedTrace.top();

    if (state->m_pimpl->m_logParsing)
    {
        // Add here the optional text content
        std::string message = std::string("Content of tag: ") + element->getParsedTextContent();
        reportInfo("XMLParser", "parserCallbackEndTag", message.c_str());

        // TODO: reportXXX should either accept a format + var arguments or something else
        message = std::string("End of tag <") + element->name() + (">");
        reportInfo("XMLParser", "parserCallbackEndTag", message.c_str());
    }
    element->exitElementScope();
    state->m_pimpl->m_parsedTrace.pop();
    // Get a callback also to the parent, if any
    if (!state->m_pimpl->m_parsedTrace.empty())
    {
        state->m_pimpl->m_parsedTrace.top()->childHasBeenParsed(element);
    }
}

void XMLParser::XMLParserPimpl::parserCallbackCharacters(void* context, const xmlChar* ch, int len)
{
    // TODO: manage better the characters.. whitespace characters & c
    XMLParser* state = static_cast<XMLParser*>(context);
    std::shared_ptr<XMLElement> element = state->m_pimpl->m_parsedTrace.top();
    std::string parsedString = std::string(reinterpret_cast<const char*>(ch), len);
    if (state->m_pimpl->m_logParsing)
    {
        std::cerr << "Ch:(" << len << ") __" << parsedString << "__" << std::endl;
    }
    element->parsedCharacters(parsedString);
}

void XMLParser::XMLParserPimpl::parserErrorMessageCallback(void* /*context*/,
                                                           const char* message,
                                                           ...)
{
    va_list arguments;
    va_start(arguments, message);
    std::string errorMessage = stringFromFormattedCString(message, arguments);
    va_end(arguments);
    reportError("XMLParser", "[Parsing]", errorMessage.c_str());
}

void XMLParser::XMLParserPimpl::parserWarningMessageCallback(void* /*context*/,
                                                             const char* message,
                                                             ...)
{
    va_list arguments;
    va_start(arguments, message);
    std::string errorMessage = stringFromFormattedCString(message, arguments);
    va_end(arguments);
    reportWarning("XMLParser", "[Parsing]", errorMessage.c_str());
}

// MARK: - XMLParser definition

XMLParser::XMLParser()
    : m_pimpl(new XMLParserPimpl())
{
    assert(m_pimpl);
    m_pimpl->f_documentFactory = [](XMLParserState& state) {
        return std::shared_ptr<XMLDocument>(new XMLDocument(state));
    };
    m_pimpl->m_performValidation = false;
    m_pimpl->m_logParsing = false;
    m_pimpl->m_keepInMemory = false;
}

XMLParser::~XMLParser()
{
}

void XMLParser::setDocumentFactory(
    std::function<std::shared_ptr<XMLDocument>(XMLParserState& state)> factory)
{
    assert(m_pimpl);
    if (factory)
    {
        m_pimpl->f_documentFactory = factory;
    } else
    {
        // Restore the default function
        m_pimpl->f_documentFactory = [](XMLParserState& state) {
            return std::shared_ptr<XMLDocument>(new XMLDocument(state));
        };
    }
}

bool XMLParser::parseXMLFile(std::string absoluteFileName)
{
    assert(m_pimpl);
    LIBXML_TEST_VERSION

    // Reset the failure state.
    m_pimpl->m_parserState.resetState();

    if (m_pimpl->m_performValidation)
    {
        if (m_pimpl->m_schemaLocation.empty())
        {
            reportError("XMLParser",
                        "parseXMLFile",
                        "Validation requested, but no schema has been specified");
            return false;
        }
        // Create necessary structures for the validator
        xmlSchemaParserCtxtPtr schemaParserContext
            = xmlSchemaNewParserCtxt(m_pimpl->m_schemaLocation.c_str());
        xmlSchemaPtr xmlSchema = xmlSchemaParse(schemaParserContext);
        xmlSchemaValidCtxtPtr validator = xmlSchemaNewValidCtxt(xmlSchema);

        // This is the actual validation. Note that there is the possibility to use
        // xmlSchemaSAXPlug which should perform the validation while parsing.
        // The code remains cleared in this way though, as the validation is optional.

        int validation = xmlSchemaValidateFile(validator, absoluteFileName.c_str(), 0);

        xmlSchemaFreeValidCtxt(validator);
        xmlSchemaFree(xmlSchema);
        xmlSchemaFreeParserCtxt(schemaParserContext);
        if (validation)
        {
            std::string message = std::string("Failed to validate ") + absoluteFileName
                                  + " for schema " + m_pimpl->m_schemaLocation;
            reportError("XMLParser", "parseXMLFile", message.c_str());
            return false;
        }
    }

    //        xmlParserCtxtPtr parserContext = xmlCreatePushParserCtxt(m_pimpl->callbackHandler(),
    //                                                                 this, NULL, 0,
    //                                                                 absoluteFileName.c_str());

    int result = xmlSAXUserParseFile(m_pimpl->callbackHandler(), this, absoluteFileName.c_str());
    return result == 0 && !m_pimpl->m_parserState.getParsingErrorState();
}

bool XMLParser::parseXMLString(std::string xmlString)
{
    assert(m_pimpl);
    LIBXML_TEST_VERSION

    // Reset the failure state.
    m_pimpl->m_parserState.resetState();

    // For now we do not support validation on the fly

    int result = xmlSAXUserParseMemory(m_pimpl->callbackHandler(),
                                       this,
                                       xmlString.c_str(),
                                       xmlString.length());
    return result == 0 && !m_pimpl->m_parserState.getParsingErrorState();
}

std::shared_ptr<const XMLDocument> XMLParser::document() const
{
    assert(m_pimpl);
    return m_pimpl->m_document;
}

bool XMLParser::validateXML() const
{
    return m_pimpl->m_performValidation;
}
void XMLParser::setValidateXML(bool validate)
{
    m_pimpl->m_performValidation = validate;
}

std::string XMLParser::schemaLocation() const
{
    return m_pimpl->m_schemaLocation;
}
void XMLParser::setSchemaLocation(std::string schemaLocation)
{
    m_pimpl->m_schemaLocation = schemaLocation;
}

bool XMLParser::logParsing() const
{
    return m_pimpl->m_logParsing;
}
void XMLParser::setLogParsing(bool enableLogging)
{
    m_pimpl->m_logParsing = enableLogging;
}

bool XMLParser::keepTreeInMemory() const
{
    return m_pimpl->m_keepInMemory;
}
void XMLParser::setKeepTreeInMemory(bool keepTreeInMemory)
{
    m_pimpl->m_keepInMemory = keepTreeInMemory;
}

void XMLParser::setPackageDirs(const std::vector<std::string>& packageDirs)
{
    m_pimpl->m_packageDirs = packageDirs;
}
const std::vector<std::string>& XMLParser::packageDirs() const
{
    return m_pimpl->m_packageDirs;
}

void XMLParserState::setParsingError()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_parsing_error = true;
}

void XMLParserState::resetState()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_parsing_error = false;
}

bool XMLParserState::getParsingErrorState()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_parsing_error;
}
} // namespace iDynTree
