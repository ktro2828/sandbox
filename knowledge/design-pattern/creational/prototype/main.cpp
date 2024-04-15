#include <string>
#include <iostream>
#include <unordered_map>
#include <functional>

enum Type
{
    PROTOTYPE_1 = 0,
    PROTOTYPE_2 = 1
};

/**
 * The example class that has cloning ability. We'll see how the values of field
 * with different types will be cloned.
 */
class Prototype
{
protected:
    std::string prototype_name_;
    float prototype_field_;

public:
    Prototype() {}
    Prototype(const std::string &prototype_name) : prototype_name_(prototype_name) {}

    virtual ~Prototype() {}
    virtual Prototype *Clone() const = 0;
    virtual void Method(float prototype_field)
    {
        this->prototype_field_ = prototype_field;
        std::cout << "Call Method from " << prototype_name_ << " with field : " << prototype_field << std::endl;
    }
}; // class Prototype

/**
 *
 *
 */
class ConcretePrototype1 : public Prototype
{
private:
    float concrete_prototype_field1_;

public:
    ConcretePrototype1(const std::string &prototype_name, float prototype_field) : Prototype(prototype_name), concrete_prototype_field1_(prototype_field) {}

    /**
     * Notice that Clone method return a Pointer to a new ConcretePrototype1
     * replica. So, the client has the responsibility to free that memory.
     * If you have smart pointer knowledge you may prefer to use `unique_ptr`
     * here.
     */
    Prototype *Clone() const override
    {
        return new ConcretePrototype1(*this);
    }
}; // class ConcretePrototype1

class ConcretePrototype2 : public Prototype
{
private:
    float concrete_prototype_field2_;

public:
    ConcretePrototype2(const std::string &prototype_name, float concrete_prototype_field) : Prototype(prototype_name), concrete_prototype_field2_(concrete_prototype_field) {}

    Prototype *Clone() const override
    {
        return new ConcretePrototype2(*this);
    }
}; // class ConcretePrototype2

/**
 * In PrototypeFactory you have two concrete prototypes, one for each concrete
 * prototype class, so each time you want to create a bullet, you can use the
 * existing ones and clone those.
 *
 * In this example, PrototypeFactory behave like prototype-registry.
 */
class PrototypeFactory
{
private:
    std::unordered_map<Type, Prototype *, std::hash<int>> prototypes_;

public:
    PrototypeFactory()
    {
        prototypes_[Type::PROTOTYPE_1] = new ConcretePrototype1("PROTOTYPE_1", 50.f);
        prototypes_[Type::PROTOTYPE_2] = new ConcretePrototype2("PROTOTYPE_2", 60.f);
    }

    /**
     * Be careful of free all memory allocated. Again, if you have smart pointers
     * knowledge will be better to use it here.
     */
    ~PrototypeFactory()
    {
        delete prototypes_[Type::PROTOTYPE_1];
        delete prototypes_[Type::PROTOTYPE_2];
    }

    /**
     * Notice here that you just need to specify the type of the prototype you
     * want and the method will create from the object with this type.
     */
    Prototype *CreatePrototype(Type type)
    {
        return prototypes_[type]->Clone();
    }
}; // class PrototypeFactory

void Client(PrototypeFactory &prototype_factory)
{
    std::cout << "Let's create a Prototype 1\n";

    Prototype *prototype = prototype_factory.CreatePrototype(Type::PROTOTYPE_1);
    prototype->Method(90.f);
    delete prototype;

    std::cout << "\n";

    std::cout << "Let's create a Prototype 2\n";

    prototype = prototype_factory.CreatePrototype(Type::PROTOTYPE_2);
    prototype->Method(10.f);
    delete prototype;
}

int main()
{
    PrototypeFactory *prototype_factory = new PrototypeFactory();
    Client(*prototype_factory);
    delete prototype_factory;

    return 0;
}