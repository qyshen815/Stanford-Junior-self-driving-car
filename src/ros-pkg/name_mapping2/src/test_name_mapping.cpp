#include <name_mapping2/name_mapping2.h>
#include <gtest/gtest.h>

using namespace std;

TEST(NameMapping2, Assigment)
{
  NameMapping2 nm;
  nm.addName("foo");
  nm.addName("bar");
  nm.addName("baz");

  cout << nm.status() << endl;
  NameMapping2 nm2 = nm;
  cout << nm2.status() << endl;
  
  EXPECT_TRUE(nm == nm2);

  NameMapping2 nm3;
  nm3 = nm;
  EXPECT_TRUE(nm == nm3);

  nm3.addName("aoeu");
  EXPECT_FALSE(nm == nm3);

  nm2.setOffset(1);
  cout << nm2.status() << endl;
  cout << nm2 << endl;
  EXPECT_FALSE(nm2 == nm);
}

TEST(NameMapping2, Serialization)
{
  NameMapping2 nm;
  nm.addName("foo");
  nm.addName("bar");
  nm.addName("baz");
  nm.setOffset(2);

  string filename = ".test-18923748736458927397653.nm";
  nm.save(filename);

  NameMapping2 nm2;
  nm2.load(filename);
  cout << nm.status() << endl;
  cout << nm << endl;
  cout << nm2.status() << endl;
  cout << nm2 << endl;
  EXPECT_TRUE(nm == nm2);

  int retval = system(("rm " + filename).c_str()); retval--;
}

TEST(NameMapping2, SpecialLabels)
{
  NameMapping2 class_map;
  class_map.addName("unlabeled");
  class_map.addName("background");
  class_map.addName("car");
  class_map.addName("pedestrian");
  class_map.addName("bicyclist");
  class_map.setOffset(2);

  EXPECT_TRUE(class_map.toName(-2).compare("unlabeled") == 0);
  EXPECT_TRUE(class_map.toName(-1).compare("background") == 0);
  EXPECT_TRUE(class_map.toName(0).compare("car") == 0);
  EXPECT_TRUE(class_map.toName(1).compare("pedestrian") == 0);
  EXPECT_TRUE(class_map.toName(2).compare("bicyclist") == 0);

  cout << class_map.toName(-2) << endl;
  cout << class_map.toName(-1) << endl;
  cout << class_map.toName(1) << endl;
}

TEST(NameMapping2, AdditionWithSpecialLabels)
{
  NameMapping2 initial;
  initial.addName("unlabeled");
  initial.addName("background");
  initial.addName("car");
  initial.addName("pedestrian");
  initial.addName("bicyclist");
  initial.setOffset(2);

  NameMapping2 supplemental;
  supplemental.addName("unlabeled");
  supplemental.addName("background");
  supplemental.addName("car");
  supplemental.addName("motorcyclist");
  supplemental.setOffset(2);

  NameMapping2 final = initial + supplemental;
  cout << final.status() << endl;
  cout << final << endl;
  EXPECT_TRUE(final.size() == 6);
  EXPECT_TRUE(final.toName(3).compare("motorcyclist") == 0);
}

TEST(NameTranslator2, Translation)
{
  NameMapping2 initial;
  initial.addName("unlabeled");
  initial.addName("background");
  initial.addName("car");
  initial.addName("pedestrian");
  initial.addName("bicyclist");
  initial.setOffset(2);

  NameMapping2 final;
  final.addName("unlabeled");
  final.addName("background");
  final.addName("bicyclist");
  final.addName("car");
  final.setOffset(2);

  NameTranslator2 translator(initial, final);
  cout << translator.status() << endl;

  EXPECT_TRUE(translator.oldToNewId(0) == 1);
  cout << translator.oldToNewId(1) << " vs " << NameTranslator2::NO_ID << endl;
  EXPECT_TRUE(translator.oldToNewId(1) == NameTranslator2::NO_ID);
}

class Foo : public NameMappable
{
public:
  NameMapping2 mapping_;
  std::vector<int> labels_;
  static Foo getTestObject();
  void applyNameMapping(const NameTranslator2& translator,
			const NameMapping2& new_mapping,
			int id = 0);
  void applyNameMapping(const NameMapping2& new_mapping,	int id = 0);
  std::string status() const;
};

void Foo::applyNameMapping(const NameTranslator2& translator,
			   const NameMapping2& new_mapping,
			   int id)
{
  for(size_t i = 0; i < labels_.size(); ++i) {
    if(translator.oldToNewId(labels_[i]) == NameTranslator2::NO_ID)
      labels_[i] = -1;
    else
      labels_[i] = translator.oldToNewId(labels_[i]);
  }
  mapping_ = new_mapping;
}

void Foo::applyNameMapping(const NameMapping2& new_mapping, int id)
{
  NameMappable::applyNameMapping(mapping_, new_mapping, id);
}

string Foo::status() const
{
  ostringstream oss;
  for(size_t i = 0; i < labels_.size(); ++i) {
    oss << i << " " << labels_[i] << " " << mapping_.toName(labels_[i]) << endl;
  }
  return oss.str();
}

Foo Foo::getTestObject()
{
  Foo foo;
  foo.mapping_.addName("unlabeled");
  foo.mapping_.addName("background");
  foo.mapping_.addName("car");
  foo.mapping_.addName("pedestrian");
  foo.mapping_.addName("bicyclist");
  foo.mapping_.setOffset(2);
  foo.labels_.push_back(0);
  foo.labels_.push_back(0);
  foo.labels_.push_back(2);
  foo.labels_.push_back(1);
  foo.labels_.push_back(-1);
  foo.labels_.push_back(-1);
  foo.labels_.push_back(-2);

  return foo;
}

TEST(NameMappable, Translation)
{
  NameMapping2 new_mapping;
  new_mapping.addName("unlabeled");
  new_mapping.addName("background");
  new_mapping.addName("pedestrian");
  new_mapping.addName("bicyclist");
  new_mapping.addName("motorcyclists");
  new_mapping.setOffset(2);

  Foo foo = Foo::getTestObject();
  cout << foo.status() << endl;
  Foo copy = foo;
  foo.NameMappable::applyNameMapping(foo.mapping_, new_mapping);
  EXPECT_TRUE(foo.labels_[0] == -1);
  EXPECT_TRUE(foo.labels_[1] == -1);
  EXPECT_TRUE(foo.labels_[2] == 1);
  EXPECT_TRUE(foo.labels_[3] == 0);
  EXPECT_TRUE(foo.labels_[4] == -1);
  EXPECT_TRUE(foo.labels_[5] == -1);
  EXPECT_TRUE(foo.labels_[6] == -2);
  cout << foo.status() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
