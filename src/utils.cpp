#include <./include/drone/utils.hpp>

namespace Utils
{
    std::vector<String> splitString(String str, char delimiter)
    {
        std::vector<String> tokens;
        int start = 0;
        int end = str.indexOf(delimiter);

        while (end != -1)
        {
            tokens.push_back(str.substring(start, end));
            start = end + 1;
            end = str.indexOf(delimiter, start);
        }

        tokens.push_back(str.substring(start));

        return tokens;
    }
}