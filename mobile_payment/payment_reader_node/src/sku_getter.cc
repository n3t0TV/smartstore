#include "sku_getter.h"

static std::string sku_;


void
mss_worldpay::SetContainerSku(std::string sku)
{
    sku_ = sku;
}


std::string
mss_worldpay::GetContainerSku(void)
{
    return sku_;
}
