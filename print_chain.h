#ifndef PRINT_CHAIN_H
#define PRINT_CHAIN_H

#include <iostream>

void print_chain_pos(joint chain[], int chain_length)
{
    cout << "Positions:" << endl << endl;
    for (int index = 0; index < chain_length; ++index)
    {
        cout << chain[index].position << endl << endl;
    }
    cout << endl << endl;
}

void print_chain_tan(joint chain[], int chain_length)
{
    cout << "Tangents:" << endl << endl;
    for (int index = 0; index < chain_length - 1; ++index)
    {
        cout << chain[index].tangent << endl << endl;
    }
    cout << endl << endl;
}

void print_chain_triad(joint chain[], int chain_length)
{
    cout << "Triads:" << endl << endl;
    for (int index = 0; index < chain_length; ++index)
    {
        cout << chain[index].triad << endl << endl;
    }
}

#endif // PRINT_CHAIN_H

