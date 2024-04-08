#include <string>
#include <iostream>

/**
 * Each distinct product of a product family should have a base interface.
 * All variants of the product must implements this interface.
 */
class AbstractProductA
{
public:
    virtual ~AbstractProductA() {}
    virtual std::string UsefulFunctionA() const = 0;
}; // class AbstractProductA

/**
 * Concrete Products are created by corresponding Concrete Factories.
 */
class ConcreteProductA1 : public AbstractProductA
{
public:
    std::string UsefulFunctionA() const override
    {
        return "The result of the product A1.";
    }
}; // class ConcreteProductA1

class ConcreteProductA2 : public AbstractProductA
{
public:
    std::string UsefulFunctionA() const override
    {
        return "The result of the product A2.";
    }
}; // class ConcreteProductA2

/**
 * Here's the base interface of another product.
 * All products can interact with each other, but proper interaction is possible only between
 * products of the same concrete variant.
 */
class AbstractProductB
{
public:
    virtual ~AbstractProductB() {}
    virtual std::string UsefulFunctionB() const = 0;
    /**
     * ...but it also can collaborate with the ProductA.
     *
     * The Abstract Factory makes sure that all products it creates are of the
     * same variant and thus, compatible.
     */
    virtual std::string AnotherUsefulFunctionB(const AbstractProductA &collaborator) const = 0;
}; // class AbstractProductB

/**
 * Concrete Products are create by corresponding Concrete Factories.
 */
class ConcreteProductB1 : public AbstractProductB
{
public:
    std::string UsefulFunctionB() const override
    {
        return "The result of the product B1.";
    }

    std::string AnotherUsefulFunctionB(const AbstractProductA &collaborator) const override
    {
        const std::string result = collaborator.UsefulFunctionA();
        return "The result of the B1 collaborating with (" + result + " )";
    }
}; // class ConcreteProductB1

class ConcreteProductB2 : public AbstractProductB
{
public:
    std::string UsefulFunctionB() const override
    {
        return "The result of the product B2.";
    }

    std::string AnotherUsefulFunctionB(const AbstractProductA &collaborator) const override
    {
        const std::string result = collaborator.UsefulFunctionA();
        return "The result of the B2 collaborating with (" + result + " )";
    }
}; // class ConcreteProductB2

/**
 * The Abstract Factory interface declares a set of methods that return
 * different abstract products. These products are called a family and are
 * related by a high-level theme or concept.
 * Products of one family are usually able to collaborate among themselves.
 * A family of products may have several variants, but the products of one variant are
 * incompatible with products of another.
 */
class AbstractFactory
{
public:
    virtual AbstractProductA *CreateProductA() const = 0;
    virtual AbstractProductB *CreateProductB() const = 0;
};

class ConcreteFactory1 : public AbstractFactory
{
public:
    AbstractProductA *CreateProductA() const override
    {
        return new ConcreteProductA1();
    }

    AbstractProductB *CreateProductB() const override
    {
        return new ConcreteProductB1;
    }
}; // class ConcreteFactory1

class ConcreteFactory2 : public AbstractFactory
{
public:
    AbstractProductA *CreateProductA() const override
    {
        return new ConcreteProductA2();
    }

    AbstractProductB *CreateProductB() const override
    {
        return new ConcreteProductB2();
    }
}; // class ConcreteFactory2

/**
 * The client code works with factories and products only through abstract
 * types: AbstractFactory and AbstractProduct.
 * This lets you pass any factory or product subclass to the client code without breaking it.
 */
void ClientCode(const AbstractFactory &factory)
{
    const AbstractProductA *product_a = factory.CreateProductA();
    const AbstractProductB *product_b = factory.CreateProductB();
    std::cout << product_b->UsefulFunctionB() << "\n";
    std::cout << product_b->AnotherUsefulFunctionB(*product_a) << "\n";
    delete product_a;
    delete product_b;
}

int main()
{
    std::cout << "Client: Testing client code with the first factory type:\n";
    ConcreteFactory1 *f1 = new ConcreteFactory1();
    ClientCode(*f1);
    delete f1;
    std::cout << std::endl;

    std::cout << "Client: Testing client code with the second factory type:\n";
    ConcreteFactory2 *f2 = new ConcreteFactory2();
    ClientCode(*f2);
    delete f2;
    return 0;
}