#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_factory
#include <boost/test/unit_test.hpp>

#include <stdio.h>
#include <robotics_toolkit/design_patterns/Factory.h>

namespace rt = robotics_toolkit;
namespace dp = rt::design_patterns;

// Define a bunch of classes that have constructors requiring
// 0 args, 1 arg, 2 args, 3 args, and 4 args

// 0 args
class ZeroArg
{
public:
  ZeroArg()
  {
    type_id = "ZeroArg";
  }

  std::string type_id;
};

// 1 arg
class OneArg
{
public:
  OneArg(const float first_)
  {
    type_id = "OneArg";
    first = first_;
  }

  std::string type_id;
  float first;
};

// 2 args
class TwoArg
{
public:
  TwoArg(const float first_,
         const float second_)
  {
    type_id = "TwoArg";
    first = first_;
    second = second_;
  }

  std::string type_id;
  float first;
  float second;

};

// 3 args
class ThreeArg
{
public:
  ThreeArg(const float first_,
           const float second_,
           const float third_)
  {
    type_id = "ThreeArg";
    first = first_;
    second = second_;
    third = third_;
  }

  std::string type_id;
  float first;
  float second;
  float third;

};

// 4 args
class FourArg
{
public:
  FourArg(const float first_,
          const float second_,
          const float third_,
          const float fourth_)
  {
    type_id = "FourArg";
    first = first_;
    second = second_;
    third = third_;
    fourth = fourth_;
  }

  std::string type_id;
  float first;
  float second;
  float third;
  float fourth;

};

BOOST_AUTO_TEST_CASE(test_factory)
{
  namespace dp = robotics_toolkit::design_patterns;
  dp::Factory& f = dp::Factory::instance();

  boost::shared_ptr<ZeroArg> a  = f.create<ZeroArg>();
  boost::shared_ptr<OneArg> b   = f.create1<OneArg, float>(0.f);
  boost::shared_ptr<TwoArg> c   = f.create2<TwoArg, float, float>(1.f, 2.f);
  boost::shared_ptr<ThreeArg> d = f.create3<ThreeArg, float, float, float>(3.f, 4.f, 5.f);
  boost::shared_ptr<FourArg> e  = f.create4<FourArg, float, float, float, float>(6.f, 7.f, 8.f, 9.f);

  // Make sure the factory created the types that we expect
  BOOST_CHECK_EQUAL(a->type_id.c_str(), "ZeroArg");
  BOOST_CHECK_EQUAL(b->type_id.c_str(), "OneArg");
  BOOST_CHECK_EQUAL(c->type_id.c_str(), "TwoArg");
  BOOST_CHECK_EQUAL(d->type_id.c_str(), "ThreeArg");
  BOOST_CHECK_EQUAL(e->type_id.c_str(), "FourArg");

  // Make sure the factor properly transferred construction arguments
  BOOST_CHECK_EQUAL(b->first,  0.f);
  BOOST_CHECK_EQUAL(c->first,  1.f);
  BOOST_CHECK_EQUAL(c->second, 2.f);
  BOOST_CHECK_EQUAL(d->first,  3.f);
  BOOST_CHECK_EQUAL(d->second, 4.f);
  BOOST_CHECK_EQUAL(d->third,  5.f);
  BOOST_CHECK_EQUAL(e->first,  6.f);
  BOOST_CHECK_EQUAL(e->second, 7.f);
  BOOST_CHECK_EQUAL(e->third,  8.f);
  BOOST_CHECK_EQUAL(e->fourth, 9.f);

  // Check that const factory methods work
  boost::shared_ptr<const ZeroArg> g  = f.createConst<ZeroArg>();
  boost::shared_ptr<const OneArg> h   = f.createConst1<OneArg, float>(0.f);
  boost::shared_ptr<const TwoArg> i   = f.createConst2<TwoArg, float, float>(1.f, 2.f);
  boost::shared_ptr<const ThreeArg> j = f.createConst3<ThreeArg, float, float, float>(3.f, 4.f, 5.f);
  boost::shared_ptr<const FourArg> k  = f.createConst4<FourArg, float, float, float, float>(6.f, 7.f, 8.f, 9.f);

}
