#ifndef AW_STRINGTOOLS_H
#define AW_STRINGTOOLS_H

#include <string>
#include <cstdio>
#include <vector>

namespace vlr {

namespace CStringTools {

// Konvertiert einen String zu einem Bool Wert
bool gbCBool(const std::string& str, const bool bDefault = false);
// Konvertiert einen String zu einem Integer Wert
int gnCInt(const std::string& str, const int nDefault = 0);
// Konvertiert einen String zu einem Float Wert
float gfCFloat(const std::string& str, const float fDefault = 0.0f);
// Konvertiert einen String zu einem Double Wert
double gdCDouble(const std::string& str, const double dDefault = 0.0);
// Konvertiert einen String zu einem Double Wert
long glCLong(const std::string& str, const long lDefault = 0);

// Konvertiert einen String Wert zu einem String
std::string gsCString(const std::string& var);
// Konvertiert einen Double Wert zu einem String
std::string gsCString(const double& var);
// Konvertiert einen Float Wert zu einem String
std::string gsCString(const float& var);
// Konvertiert einen Bool Wert zu einem String
std::string gsCString(const bool& var);
// Konvertiert einen Integer Wert zu einem String
std::string gsCString(const int& var);

void splitString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ");

std::string clearCComments(std::string);

}; // CStringTools

} // namespace vlr

#endif
