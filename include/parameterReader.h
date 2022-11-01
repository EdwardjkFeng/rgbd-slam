# pragma once

#include <fstream>
#include <iostream>
#include <map>
using namespace std;

class ParameterReader
{
public:
    ParameterReader(string filename = "../conf/parameters.txt")
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

            int pos = str.find("=");
            if(pos == -1)
                continue;
            string key = str.substr(0, pos);
            string value = str.substr(pos + 1 , str.length());

            data[key] = value;

            if (!fin.good())
            {
                break;
            }
            
        }
    }

    string getData(string key)
    {
        map<string, string>::iterator iter = data.find(key);

        if (iter == data.end())
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return string("NOT_FOUND");
        }

        return iter->second;
    }

    map<string, string> data;
};


inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;

    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof(pd.getData("camera.fx").c_str());
    camera.fy = atof(pd.getData("camera.fy").c_str());
    camera.cx = atof(pd.getData("camera.cx").c_str());
    camera.cy = atof(pd.getData("camera.cy").c_str());
    camera.scale = atof(pd.getData("camera.scale").c_str());

    return camera;
}
