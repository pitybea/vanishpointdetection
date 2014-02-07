#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "../FileIO/FileInOut.h"


using namespace std;

int main(int argc, char* argv[])
{
	_chdir("E:\\vanish\\dataset\\paris");

	auto imgs=fileIOclass::InVectorString("img.lst");
	return 0;
}