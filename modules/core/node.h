#ifndef CORE_NODE_H_
#define CORE_NODE_H_

class Node
{
public:
    virtual ~Node();

    virtual void Apply(Signal& signal, const Indices& indices) = 0; ///< Apply the node on indexes in this signal.


protected:
    Node();
};

#endif // CORE_NODE_H_
