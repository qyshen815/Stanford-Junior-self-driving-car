#include <name_mapping/name_mapping.h>
#include <gtest/gtest.h>

using namespace std;

TEST(NameMapping, mapping) {
  vector<string> names1;
  names1.push_back("aaa");
  names1.push_back("bbb");
  names1.push_back("ccc");
  vector<string> names2;
  names2.push_back("ccc");
  names2.push_back("bbb");
  names2.push_back("aaa");
  
  NameMapping m1(names1);
  NameMapping m2(names2);

  NameTranslator t12(m1, m2);
  for(size_t i=0; i<names1.size(); ++i) {
    EXPECT_TRUE(m1.toName(i) == m2.toName(t12.toMap2(i)));
  }
}

TEST(NameMapping, loadAndSave) {
  vector<string> names1;
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("aaa");

  NameMapping m1(names1);
  string m1str = m1.serialize();
  //  cout << m1str << endl;

  istringstream iss(m1str);
  NameMapping m2(iss);
  //  cout << m2.serialize() << endl;
  EXPECT_TRUE(m1.compare(m2));
}

TEST(NameMapping, compare) {
  vector<string> names1;
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("aaa");

  vector<string> names2;
  names2.push_back("ccc");
  names2.push_back("bbb");
  names2.push_back("xxx");

  NameMapping m1(names1);
  NameMapping m2(names2);
  
  EXPECT_TRUE(m1.compare(m1));
  EXPECT_FALSE(m1.compare(m2));
}

TEST(NameMapping, augmentation) {
  vector<string> names1;
  names1.push_back("xxx");
  names1.push_back("ccc");
  names1.push_back("bbb");
  names1.push_back("aaa");

  vector<string> names2;
  names2.push_back("ccc");


  NameMapping m1(names1);
  NameMapping m2(names2);

  m2.augment(m1);
  //cout << m1.serialize() << endl;
  //cout << m2.serialize() << endl;
  
  NameTranslator t12(m1, m2);
  for(size_t i=0; i<4; ++i) { 
    EXPECT_TRUE(m1.toName(i) == m2.toName(t12.toMap2(i)));
  }
}
  

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

  
