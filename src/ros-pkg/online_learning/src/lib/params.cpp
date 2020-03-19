#include <online_learning/online_learning.h>

using namespace std;

namespace odontomachus
{

  void Params::serialize(std::ostream& out) const
  {
    out << size() << endl;
    map<string, double>::const_iterator it;
    for(it = map<string, double>::begin(); it != end(); ++it)
      out << it->first << "\t" << setprecision(20) << it->second << endl;
  }

  void Params::deserialize(std::istream& in)
  {
    size_t num;
    in >> num;

    string sbuf;
    double dbuf;
    for(size_t i = 0; i < num; ++i) { 
      in >> sbuf;
      in >> dbuf;
      insert(pair<string, double>(sbuf, dbuf));
    }
  }

}
