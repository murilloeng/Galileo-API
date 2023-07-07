//std
#include <omp.h>

//maker
#include "path.h"

void print_file(std::string path)
{
	FILE* file = fopen(path.c_str(), "r");
	while(!feof(file))
	{
		printf("%c", fgetc(file));
	}
	fclose(file);
}
void join_logs(std::string log_file)
{
	char cmd[200];
	char log_path[50];
	bool join = false;
	for(unsigned i = 0; i < omp_get_num_threads(); i++)
	{
		sprintf(log_path, "build\\log_%02d.txt", i);
		join = join || std::filesystem::exists(log_path);
	}
	if(log_file.empty()) log_file = "build\\log.txt";
	sprintf(cmd, "copy /b build\\log_*.txt %s > nul & del build\\log_*.txt", log_file.c_str());
	if(join) system(cmd);
}

std::filesystem::path uic_to_inc(std::filesystem::path path)
{
	path = replace(path, ".ui", ".h");
	path = replace(path, "ui/", "build/uic/");
	return path;
}
std::filesystem::path inc_to_src(std::filesystem::path path)
{
	path = replace(path, ".h", ".cpp");
	path = replace(path, "inc/", "build/moc/");
	return path;
}
std::filesystem::path rsc_to_src(std::filesystem::path path)
{
	path = replace(path, ".qrc", ".cpp");
	path = replace(path, "rsc/", "build/rsc/", 1);
	return path;
}
std::filesystem::path inc_to_obj(std::filesystem::path path, std::string mode)
{
	path = replace(path, ".h", ".obj");
	path = replace(path, "inc/", "build/" + mode + "/moc/");
	return path;
}
std::filesystem::path src_to_obj(std::filesystem::path path, std::string mode)
{
	path = replace(path, ".cpp", ".obj");
	path = replace(path, "src/", "build/" + mode + "/obj/");
	return path;
}
std::filesystem::path rsc_to_obj(std::filesystem::path path, std::string mode)
{
	path = replace(path, ".qrc", ".obj");
	path = replace(path, "rsc/", "build/" + mode + "/rsc/", 1);
	return path;
}
std::filesystem::path ext_to_dll(std::filesystem::path path, std::string mode)
{
	return replace(path, "../ext/dll/x64/", "dist/" + mode + "/");
}

std::string& strip(std::string& str, std::string s1)
{
	return replace(str, s1, "");
}
std::string& replace(std::string& str, std::string s1, std::string s2, unsigned max)
{
	for(unsigned i = 0; i < max; i++)
	{
		size_t index = str.find(s1);
		if(index == std::string::npos) break;
		str.replace(index, s1.length(), s2);
	}
	return str;
}
std::filesystem::path& replace(std::filesystem::path& path, std::string s1, std::string s2, unsigned max)
{
	std::string str = path.string();
	for(unsigned i = 0; i < max; i++)
	{
		size_t index = str.find(s1);
		if(index == std::string::npos) break;
		str.replace(index, s1.length(), s2);
	}
	path = str;
	return path;
}

void dep_list(std::filesystem::path path, std::vector<std::filesystem::path>& list, const std::vector<std::filesystem::path>& dirs)
{
	//open
	FILE* file = fopen(path.string().c_str(), "r");
	//read
	char buffer[1000];
	while(!feof(file))
	{
		fgets(buffer, 1000, file);
		if(std::string(buffer)._Starts_with("#include \""))
		{
			std::string str = buffer;
			strip(strip(strip(str, "#include "), "\""), "\n");
			for(const std::filesystem::path& dir : dirs)
			{
				if(std::filesystem::exists(dir.string() + str))
				{
					list.push_back(dir.string() + str);
					dep_list(dir.string() + str, list, dirs);
				}
			}
		}
	}
	std::sort(list.begin(), list.end() );
	list.erase(std::unique(list.begin(), list.end()), list.end());
	//close
	fclose(file);
}