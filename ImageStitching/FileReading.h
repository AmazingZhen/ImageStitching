#ifndef _FILEREADING_H_
#define _FILEREADING_H_

#include <io.h>
#include <string>
#include <vector>
#include <iostream>

using namespace std;

void getAllFiles(string path, vector<string> &files);
void getAllFormatFiles(string path, vector<string>& files, string format);

#endif