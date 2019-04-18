#ifndef UTILS_HPP_INCLUDED
#define UTILS_HPP_INCLUDED

#include <string>

namespace Utils
{
    //  default console log tag
    static std::string ERROR("\033[0;31mERROR\033[0m");
    static std::string WARNING("\033[0;33mWARNING\033[0m");
    static std::string SUCCESS("\033[0;32mSUCCESS\033[0m");
    static std::string INFOS("\033[0;36mERROR\033[0m");

    //static std::string printRed(const std::string& s) { return "\033[0;31m" + s + "\033[0m"; };
    //static std::string printYellow(const std::string& s) { return "\033[0;33m" + s + "\033[0m"; };
    //static std::string printGreen(const std::string& s) { return "\033[0;32m" + s + "\033[0m"; };
    //static std::string printCyan(const std::string& s) { return "\033[0;36m" + s + "\033[0m"; };
};


#endif // UTILS_HPP_INCLUDED
