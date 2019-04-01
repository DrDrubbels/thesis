#ifndef FILES_H
#define FILES_H

#include <fstream>
#include <sys/stat.h>
#include <windows.h>

using namespace std;

void create_file(string filename)
{
    ofstream myfile;
    myfile.open(filename);
    myfile << "";
    myfile.close();
}

void append_file(string filename, string contents)
{
    ofstream myfile;
    myfile.open(filename,ios_base::app);
    myfile << contents;
    myfile.close();
}

bool DoesFileExist (const string& name)
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

string next_filename(int start_nr, string directory, string extension)
{
    bool already_exists = true;
    int file_nr = start_nr;
    do
    {
        string filename = directory + "\\" + to_string(file_nr) + extension;
        if (DoesFileExist(filename))
        {
            // cout << filename << " already exists :(" << endl;
            ++file_nr;
        }
        else
        {
            // cout << filename << " does not yet exist :D" << endl;
            already_exists = false;
        }
    }
    while (already_exists);

    string filename = to_string(file_nr) + extension;
    return directory + "\\" + filename;
}

#endif // FILES_H



