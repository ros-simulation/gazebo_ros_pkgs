/// File to use sdf::Element::Get method providing a vector of bools

#pragma once

#include <iostream>
#include <vector>

class bool_array
{
private:
    std::vector<bool> _bool_vec;
public:
    bool_array(/* args */);
    ~bool_array();
    friend std::istream& operator>>( std::istream& _out, const bool_array& _pt);
    void push_back(bool const bool_var);
    std::vector<bool> get();
};

bool_array::bool_array(/* args */)
{
    _bool_vec.resize(0);
}

bool_array::~bool_array()
{
}

void bool_array::push_back(bool const bool_var)
{
    _bool_vec.push_back(bool_var);
}

std::vector<bool> bool_array::get()
{
    return _bool_vec;
}

std::istream& operator>>( std::istream& _in, bool_array& _pt)
{
    // Skip white spaces
    bool bool_var;
    _in.setf(std::ios_base::skipws);
    while (_in >> bool_var)
    {
        _pt.push_back(bool_var);
    }
    return _in;
}