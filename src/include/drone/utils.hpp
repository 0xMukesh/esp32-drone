#pragma once

#include <Arduino.h>
#include <vector>

namespace Utils
{
    std::vector<String> splitString(String str, char delimiter);
}
