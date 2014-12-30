#include "core/registor.h"

#include <boost/assign/list_of.hpp>

#include "core/exception.h"
#include "ts/ts.h"

namespace {

using boost::assign::map_list_of;

enum ClassId { B, X, Y }; ///< for identifying child type inside tests

struct Base {

    int x;
    virtual ClassId Id() const { return B; }
};

struct DX : public Base {
    virtual ClassId Id() const { return X; }
};

struct DY : public Base {
    virtual ClassId Id() const { return Y; }
};

typedef Registor_<Base> DummyRegistor;
typedef DummyRegistor::Registry DummyRegistry;
typedef std::shared_ptr<Base> InstancePtrShared;

// macros for adding individual instances to registry
#define ADD_TO_REGISTRY(Registor, NewInstance) (#NewInstance, &Registor::DerivedInstance<NewInstance>)
#define ADD_TO_DUMMY_REGISTRY(NewInstance) ADD_TO_REGISTRY(DummyRegistor, NewInstance)

DummyRegistry g_dummyRegistry = map_list_of
        ADD_TO_DUMMY_REGISTRY( DX )
        ADD_TO_DUMMY_REGISTRY( DY )
        ;

class RegistorTest : public ::testing::Test
{
};

TEST_F(RegistorTest, RegistrySize)
{
    EXPECT_SIZE(2, g_dummyRegistry) << "Not all instance types were registered";
}

TEST_F(RegistorTest, Find)
{
    EXPECT_TRUE(DummyRegistor::Find(g_dummyRegistry, "DX"));
    EXPECT_TRUE(DummyRegistor::Find(g_dummyRegistry, "DY"));
    EXPECT_FALSE(DummyRegistor::Find(g_dummyRegistry, "Base")) << "Base intentionally not added";
    EXPECT_FALSE(DummyRegistor::Find(g_dummyRegistry, "wrong")) << "Never add this one.";
    EXPECT_FALSE(DummyRegistor::Find(g_dummyRegistry, ""));
}

TEST_F(RegistorTest, EmptyRegistry)
{
    EXPECT_FALSE(DummyRegistor::Find(DummyRegistry(), "DX"));
    EXPECT_FALSE(DummyRegistor::Find(DummyRegistry(), ""));
}

TEST_F(RegistorTest, DerivedInstance)
{
    InstancePtrShared ptr;

    ptr = DummyRegistor::DerivedInstance<DX>();
    EXPECT_EQ(ptr->Id(), ClassId::X);

    ptr = DummyRegistor::DerivedInstance<DY>();
    EXPECT_EQ(ptr->Id(), ClassId::Y);
}

TEST_F(RegistorTest, CreateSharedPtr)
{
    {
        InstancePtrShared ptr = DummyRegistor::CreatePtrShared(g_dummyRegistry, "DX");
        EXPECT_EQ(ptr->Id(), ClassId::X);
    }
    {
        InstancePtrShared ptr = DummyRegistor::CreatePtrShared(g_dummyRegistry, "DY");
        EXPECT_EQ(ptr->Id(), ClassId::Y);
    }
}

TEST_F(RegistorTest, CreateSharedPtr_WrongType)
{
    EXPECT_THROW(DummyRegistor::CreatePtrShared(g_dummyRegistry, "Blahbla"), sem::ExceptionTypeError);
}

TEST_F(RegistorTest, CreateSharedPtr_UniqueInstancesSameType)
{
    const std::string TYPE="DY";

    InstancePtrShared ptr1 = DummyRegistor::CreatePtrShared(g_dummyRegistry, TYPE);
    InstancePtrShared ptr2 = DummyRegistor::CreatePtrShared(g_dummyRegistry, TYPE);

    ptr1->x = 55;
    ASSERT_EQ(55, ptr1->x);

    ptr2->x = 33;
    ASSERT_EQ(33, ptr2->x);

    EXPECT_NE(ptr1, ptr2);

    EXPECT_EQ(55, ptr1->x);
}


TEST_F(RegistorTest, CreateSharedPtr_UniqueInstancesDifferentTypes)
{
    InstancePtrShared ptr1 = DummyRegistor::CreatePtrShared(g_dummyRegistry, "DX");
    ptr1->x = 55;
    ASSERT_EQ(55, ptr1->x);

    InstancePtrShared ptr2 = DummyRegistor::CreatePtrShared(g_dummyRegistry, "DY");
    ptr2->x = 33;
    ASSERT_EQ(33, ptr2->x);

    EXPECT_NE(ptr1, ptr2);

    EXPECT_EQ(55, ptr1->x);

    EXPECT_EQ(ptr1->Id(), ClassId::X);
    EXPECT_EQ(ptr2->Id(), ClassId::Y);
}

} // anonymous namespace
