#include <sstream>

#include "worldpay_parser_exception.h"
#include "worldpay_response_parser.h"


mobile_payment_interface::WorldpayTransResponse
WorldpayResponseParser::Parse(std::string xml_string)
{
    const char* kSaleResponseNodeName = "CreditCardSaleResponse";
    const char* kResponseNodeName = "Response";
    const char* kTransactionNodeName = "Transaction";
    pugi::xml_node tmp_node;

    GetXmlDocumentFromString(xml_string);
    tmp_node = GetNodeFromDoc(kSaleResponseNodeName);
    tmp_node = GetChildFromNode(kResponseNodeName, tmp_node);
    GetExpressDataFromResponseNode(tmp_node);
    tmp_node = GetChildFromNode(kTransactionNodeName, tmp_node);
    GetDataFromTransNode(tmp_node);
    wp_response_.is_mock_response = false;

    return wp_response_;
}


void
WorldpayResponseParser::GetXmlDocumentFromString(const std::string& xml_string)
{
    pugi::xml_parse_result load_ok = xml_doc_.load_string(xml_string.c_str());

    if(!load_ok)
    {
        throw WorldpayParserException("The XML string of the transaction"
                                      " response couldn't be loaded");
    }
}


pugi::xml_node
WorldpayResponseParser::GetNodeFromDoc(std::string node_name)
{
    pugi::xml_node doc_node = xml_doc_.child(node_name.c_str());

    VerifyNodeIsNotNull(doc_node, node_name);

    return doc_node;
}


pugi::xml_node
WorldpayResponseParser::GetChildFromNode(std::string child_name,
                                         pugi::xml_node parent_node)
{
    pugi::xml_node child_node = parent_node.child(child_name.c_str());

    VerifyNodeIsNotNull(child_node, child_name);

    return child_node;
}


void
WorldpayResponseParser::GetExpressDataFromResponseNode(pugi::xml_node response)
{
    const char* kResponseCodeNodeName = "ExpressResponseCode";
    const char* kResponseMessageNodeName = "ExpressResponseMessage";
    const char* kTransactionDateNodeName = "ExpressTransactionDate";
    const char* kTransactionTimeNodeName = "ExpressTransactionTime";
    const char* kTransactionTimezoneNodeName = "ExpressTransactionTimezone";
    pugi::xml_node tmp_node;

    tmp_node = GetChildFromNode(kResponseCodeNodeName, response);
    wp_response_.express_response_code = tmp_node.text().as_int();

    tmp_node = GetChildFromNode(kResponseMessageNodeName, response);
    wp_response_.express_response_message = tmp_node.child_value();

    if(wp_response_.express_response_code == wp_response_.kApprovedResponseCode)
    {
        tmp_node = GetChildFromNode(kTransactionDateNodeName, response);
        wp_response_.express_transaction_date = tmp_node.child_value();

        tmp_node = GetChildFromNode(kTransactionTimeNodeName, response);
        wp_response_.express_transaction_time = tmp_node.child_value();

        tmp_node = GetChildFromNode(kTransactionTimezoneNodeName, response);
        wp_response_.express_transaction_timezone = tmp_node.child_value();
    }
}


void
WorldpayResponseParser::GetDataFromTransNode(pugi::xml_node transaction)
{
    const char* kTransactionIdNodeName = "TransactionID";
    const char* kApprovalNumberNodeName = "ApprovalNumber";
    const char* kReferenceNumberNodeName = "ReferenceNumber";
    const char* kApprovedAmountNodeName = "ApprovedAmount";

    pugi::xml_node tmp_node;

    tmp_node = GetChildFromNode(kReferenceNumberNodeName, transaction);
    wp_response_.reference_number = tmp_node.child_value();

    if(wp_response_.express_response_code == wp_response_.kApprovedResponseCode)
    {
        tmp_node = GetChildFromNode(kTransactionIdNodeName, transaction);
        wp_response_.transaction_id = tmp_node.child_value();

        tmp_node = GetChildFromNode(kApprovalNumberNodeName, transaction);
        wp_response_.approval_number = tmp_node.child_value();

        tmp_node = GetChildFromNode(kApprovedAmountNodeName, transaction);
        wp_response_.approved_amount = tmp_node.text().as_float();
    }
}


void
WorldpayResponseParser::VerifyNodeIsNotNull(const pugi::xml_node& node,
                                            std::string node_name)
{
    if(node.type() == pugi::node_null)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The node \"" << node_name << "\""
                       << " was not found in the XML transaction response";

        throw WorldpayParserException(err_msg_stream.str());
    }
}
