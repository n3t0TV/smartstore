#pragma once

#include <pugixml.hpp>

#include "mobile_payment_interface/WorldpayTransResponse.h"


class WorldpayResponseParser
{
    public:
        mobile_payment_interface::WorldpayTransResponse
        Parse(std::string xml_string);

    private:
        pugi::xml_document xml_doc_;

        mobile_payment_interface::WorldpayTransResponse wp_response_;

        void GetXmlDocumentFromString(const std::string& xml_string);

        pugi::xml_node GetNodeFromDoc(std::string node_name);

        pugi::xml_node GetChildFromNode(std::string child_name,
                                        pugi::xml_node parent_node);

        void GetExpressDataFromResponseNode(pugi::xml_node response);

        void GetDataFromTransNode(pugi::xml_node transaction);

        void VerifyNodeIsNotNull(const pugi::xml_node& node,
                                 std::string node_name);
};
