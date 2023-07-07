//std
#include <omp.h>
#include <vector>
#include <filesystem>

//maker
#include "path.h"

//data
static bool run = false, debug = false, gui = false, clean = false;
static std::vector<std::filesystem::path> inc_paths, src_paths, dll_paths;
static std::string CXX, CXXFLAGS, LIBS, ELIB, EDLL, DEFS, INCS, LNKS, mode;

//head
void setup(int, char**);

void setup_strs(void);
void setup_mode(void);
void setup_libs(void);
void setup_dlls(void);
void setup_libs_debug(void);
void setup_libs_release(void);

//setup
void setup(int argc, char** argv)
{
	//args
	mode = "debug";
	for(unsigned i = 0; i < argc; i++)
	{
		if(strcmp(argv[i], "m=r") == 0)
		{
			mode = "release";
		}
		run = run || strcmp(argv[i], "run") == 0;
		gui = gui || strcmp(argv[i], "gui") == 0;
		debug = debug || strcmp(argv[i], "debug") == 0;
		clean = clean || strcmp(argv[i], "clean") == 0;
	}
	//dist
	if(!std::filesystem::exists("dist/" + mode))
	{
		std::filesystem::create_directories("dist/" + mode);
	}
	//setup
	setup_strs();
	setup_mode();
	setup_libs();
	setup_dlls();
}
void setup_strs(void)
{
	CXX = "cl ";
	ELIB = "../libs/";
	EDLL = "../dlls/";
	LNKS = "/IGNORE:4099 ";
	DEFS = "/D \"NOMINMAX\" /D \"_USE_MATH_DEFINES\" ";
	INCS = "/I inc /I build /I ../mat/inc /I ../fea/inc /I ../ext/inc ";
	CXXFLAGS = "/nologo /std:c++20 /EHsc /c /openmp /diagnostics:caret " + DEFS + INCS;
}
void setup_mode(void)
{
	if(mode.compare("debug") == 0)
	{
		LNKS += "/debug ";
		CXXFLAGS += "/Zi /FS ";
		CXXFLAGS += "/Fddist/debug/ben.pdb ";
		CXXFLAGS += "/D \"_DEBUG\" /Od /MDd /RTC1 ";
	}
	else
	{
		CXXFLAGS += "/D \"NDEBUG\" /O2 /MD ";
	}
}
void setup_libs(void)
{
	LIBS.clear();
	LIBS += "glu32.lib ";
	LIBS += "opengl32.lib ";
	LIBS += ELIB + "gmsh.lib ";
	LIBS += ELIB + "libblas.lib ";
	LIBS += ELIB + "liblapack.lib ";
	mode.compare("debug") == 0 ? setup_libs_debug() : setup_libs_release();
}
void setup_dlls(void)
{
	dll_paths.clear();
	dll_paths.push_back(EDLL + "libblas.dll");
	dll_paths.push_back(EDLL + "gmsh-4.9.dll");
	dll_paths.push_back(EDLL + "liblapack.dll");
	dll_paths.push_back(EDLL + "libquadmath-0.dll");
	dll_paths.push_back(EDLL + "libgfortran-3.dll");
	dll_paths.push_back(EDLL + "libgcc_s_sjlj-1.dll");
}
void setup_libs_debug(void)
{
	LIBS += ELIB + "metisd.lib ";
	LIBS += ELIB + "libmatd.lib ";
	LIBS += ELIB + "libfead.lib ";
	LIBS += ELIB + "libamdd.lib ";
	LIBS += ELIB + "libcamdd.lib ";
	LIBS += ELIB + "libcolamdd.lib ";
	LIBS += ELIB + "libccolamdd.lib ";
	LIBS += ELIB + "libcholmodd.lib ";
	LIBS += ELIB + "libumfpackd.lib ";
	LIBS += ELIB + "libquadruled.lib ";
	LIBS += ELIB + "suitesparseconfigd.lib ";
}
void setup_libs_release(void)
{
	LIBS += ELIB + "metis.lib ";
	LIBS += ELIB + "libmat.lib ";
	LIBS += ELIB + "libfea.lib ";
	LIBS += ELIB + "libamd.lib ";
	LIBS += ELIB + "libcamd.lib ";
	LIBS += ELIB + "libcolamd.lib ";
	LIBS += ELIB + "libccolamd.lib ";
	LIBS += ELIB + "libcholmod.lib ";
	LIBS += ELIB + "libumfpack.lib ";
	LIBS += ELIB + "libquadrule.lib ";
	LIBS += ELIB + "suitesparseconfig.lib ";
}

void find_inc(void)
{
	for(const std::filesystem::directory_entry& entry : std::filesystem::recursive_directory_iterator("inc/"))
	{
		const std::string ext = ".h";
		std::filesystem::path item = entry.path();
		if(item.string().compare(item.string().length() - ext.length(), ext.length(), ext) == 0)
		{
			replace(item, "\\", "/");
			inc_paths.push_back(item);
		}
	}
}
void find_src(void)
{
	for(const std::filesystem::directory_entry& entry : std::filesystem::recursive_directory_iterator("src/"))
	{
		const std::string ext = ".cpp";
		std::filesystem::path item = entry.path();
		if(item.string().compare(item.string().length() - ext.length(), ext.length(), ext) == 0)
		{
			replace(item, "\\", "/");
			src_paths.push_back(item);
		}
	}
}

void build_rsc(void)
{
	system("rc /nologo /r rsc/msvc.rc");
}
void build_src(void)
{
	omp_set_num_threads(16);
	#pragma omp parallel for
	for(int i = 0; i < src_paths.size(); i++)
	{
		//data
		char log_path[50];
		const unsigned a = omp_get_thread_num();
		std::vector<std::filesystem::path> list;
		const std::filesystem::path src = src_paths[i];
		const std::filesystem::path obj = src_to_obj(src, mode);
		std::vector<std::filesystem::path> dirs = {"inc/", "../mat/inc/", "../fea/inc/"};
		//check
		dep_list(src, list, dirs);
		bool build = !std::filesystem::exists(obj);
		sprintf(log_path, "build\\log_%02d.txt", a);
		build = build || std::filesystem::last_write_time(obj) < std::filesystem::last_write_time(src);
		for(const std::filesystem::path& inc : list)
		{
			build = build || std::filesystem::last_write_time(obj) < std::filesystem::last_write_time(inc);
		}
		const std::string cmd = CXX + " " + CXXFLAGS + " /Fo:" + obj.string() + " " + src.string() + " >> " + log_path;
		//build
		if(build)
		{
			if(!std::filesystem::exists(obj.parent_path()))
			{
				std::filesystem::create_directories(obj.parent_path());
			}
			printf("compiling(%s): %s\n", mode.c_str(), src.string().c_str());
			if(system(cmd.c_str()))
			{
				print_file(log_path);
				exit(EXIT_FAILURE);
			}
		}
	}
	join_logs();
}
void build_dll(void)
{
	omp_set_num_threads(16);
	#pragma omp parallel for
	for(int i = 0; i < dll_paths.size(); i++)
	{
		std::filesystem::path ext = dll_paths[i];
		std::filesystem::path dll = ext_to_dll(ext, mode);
		const std::string cmd = "copy /y " + replace(ext, "/", "\\").string() + " " + replace(dll, "/", "\\").string() + " > nul";
		if(!std::filesystem::exists(dll) || std::filesystem::last_write_time(dll) < std::filesystem::last_write_time(ext))
		{
			if(system(cmd.c_str())) exit(EXIT_FAILURE);
		}
	}
}
void build_exe(void)
{
	//data
	std::string objs;
	const std::string dir = "dist/" + mode;
	//objects
	for(const std::filesystem::path& path : src_paths)
	{
		objs += src_to_obj(path, mode).string() + " ";
	}
	//temp
	FILE* file = fopen("objs", "w");
	fprintf(file, "%s", objs.c_str());
	fclose(file);
	//setup
	if(!std::filesystem::exists(dir.c_str()))
	{
		std::filesystem::create_directories(dir);
	}
	//build
	std::string out = "dist/" + mode + "/ben.exe ";
	std::string cmd = "link /nologo " + LNKS + "/out:" + out + "@objs rsc/msvc.res " + LIBS;
	system(cmd.c_str());
	printf("executable(%s): %s\n", mode.c_str(), out.c_str());
	//delete
	std::filesystem::remove("objs");
}
void build_run(void)
{
	if(!run) return;
	std::filesystem::path out = "dist/" + mode + "/ben.exe ";
	system(replace(out, "/", "\\").string().c_str());
}
void build_gui(void)
{
	if(!gui) return;
	std::filesystem::current_path("../gui");
	std::filesystem::path out = "dist/" + mode + "/gui.exe ";
	system(replace(out, "/", "\\").string().c_str());
}
void build_debug(void)
{
	if(!debug) return;
	system("devenv /debugexe dist/debug/ben.exe");
}
void build_clean(void)
{
	if(std::filesystem::exists("dist/" + mode))
	{
		std::filesystem::remove_all("dist/" + mode);
	}
	if(std::filesystem::exists("build/" + mode))
	{
		std::filesystem::remove_all("build/" + mode);
	}
}

int main(int argc, char** argv)
{
	//setup
	setup(argc, argv);
	//files
	find_inc();
	find_src();
	//build
	if(!clean)
	{
		build_rsc();
		build_src();
		build_dll();
		build_exe();
		build_run();
		build_gui();
		build_debug();
	}
	else
	{
		build_clean();
	}
	//return
	return 0;
}