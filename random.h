#ifndef RANDOM_H
#define RANDOM_H

using namespace std;

double rand_normal(double average, double sigma)
{
    static bool first_call = true;
    if (first_call)
    {
        first_call = false;
        srand(2);
    }
    static default_random_engine generator;
    static normal_distribution<double> distribution(0.0,1.0);
    return average + sigma * distribution(generator);
}

double rand_double(double min_val, double max_val)
{
    static bool first_call = true;
    if (first_call)
    {
        first_call = false;
        srand(1);
    }
    return (double)rand()/RAND_MAX * (max_val - min_val) + min_val;
}

int rand_int(int min_val, int max_val) //min_val inclusive; max_val exclusive
{
    static bool first_call = true;
    if (first_call)
    {
        first_call = false;
        srand(1);
    }
    return rand() % (max_val - min_val) + min_val;
}

bool roll(double probability)
{
    double roll_outcome = rand_double(0.0,1.0);
    if (roll_outcome < probability)
    {
        // cout << "Rolled " << roll_outcome << " < " << probability << endl;
        return true;
    }else
    {
        // cout << "Rolled " << roll_outcome << " > " << probability << endl;
        return false;
    }
}

struct index_pair
{
    int index_1;
    int index_2;
};

index_pair rand_pair(int min_val, int max_val, bool distinct)
{
    int index_1 = rand_int(min_val, max_val);
    int index_2;
    if (distinct)
    {
        index_2 = rand_int(min_val, max_val - 1);
        if (index_2 >= index_1)
        {
            ++index_2;
        }
    }
    else
    {
        index_2 = rand_int(min_val, max_val);
    }
    index_pair out_pair;
    out_pair.index_1 = index_1;
    out_pair.index_2 = index_2;

    return out_pair;
}

struct crank_shaft_circular_index_pair
{
    int low_index;
    int high_index;
    bool in_between;
};

crank_shaft_circular_index_pair rand_crank_shaft_circular_pair(int chain_length)
{
    index_pair indices = rand_pair(0, chain_length, true);

    crank_shaft_circular_index_pair crankdices;
    crankdices.low_index = min(indices.index_1, indices.index_2);
    crankdices.high_index = max(indices.index_1, indices.index_2);

    if (crankdices.high_index - crankdices.low_index <= chain_length/2)
    {
        crankdices.in_between = true;
    }
    else
    {
        crankdices.in_between = false;
    }

    return crankdices;
}


#endif // RANDOM_H
