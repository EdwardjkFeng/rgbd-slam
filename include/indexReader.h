# pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <vector>
using namespace std;

class IndexReader
{
public:
    IndexReader(string filename) 
    {
        ifstream fin(filename.c_str());
        if (!fin)
        {
            cerr << "Parameter files does not exist." << endl;
            return;
        }
        
        while (!fin.eof())
        {
            string str;
            getline(fin, str);
            if (str[0] == '#')
            {
                continue;
            }

            stringstream ss(str);
            string s;
            while (getline(ss, s, ' '))
            {
                associated.push_back(s);
            }


            if (!fin.good())
            {
                break;
            }
            
        }

        for (size_t i = 0; i < (associated.size() / 4); i++)
        {
            rgb.push_back(associated[4 * i + 1]);
            depth.push_back(associated[4 * i + 3]);
        }
    }

public:
    vector<string> associated;
    vector<string> rgb;
    vector<string> depth;
};
