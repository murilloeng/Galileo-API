//maker
#include "../../win/Maker.h"

void build_rsc(void)
{
	system("rc /nologo /r rsc/msvc.rc");
}
void setup_dlls(Maker& maker)
{
	maker.m_path_dll.clear();
	maker.m_path_dll.push_back(maker.m_edll + "libblas.dll");
	maker.m_path_dll.push_back(maker.m_edll + "gmsh-4.9.dll");
	maker.m_path_dll.push_back(maker.m_edll + "liblapack.dll");
	maker.m_path_dll.push_back(maker.m_edll + "libquadmath-0.dll");
	maker.m_path_dll.push_back(maker.m_edll + "libgfortran-3.dll");
	maker.m_path_dll.push_back(maker.m_edll + "libgcc_s_sjlj-1.dll");
}
void setup_libs_debug(Maker& maker)
{
	maker.m_libs += maker.m_elib + "metisd.lib ";
	maker.m_libs += maker.m_elib + "libmatd.lib ";
	maker.m_libs += maker.m_elib + "libfead.lib ";
	maker.m_libs += maker.m_elib + "libamdd.lib ";
	maker.m_libs += maker.m_elib + "libcamdd.lib ";
	maker.m_libs += maker.m_elib + "libcolamdd.lib ";
	maker.m_libs += maker.m_elib + "libccolamdd.lib ";
	maker.m_libs += maker.m_elib + "libcholmodd.lib ";
	maker.m_libs += maker.m_elib + "libumfpackd.lib ";
	maker.m_libs += maker.m_elib + "libquadruled.lib ";
	maker.m_libs += maker.m_elib + "suitesparseconfigd.lib ";
}
void setup_libs_release(Maker& maker)
{
	maker.m_libs += maker.m_elib + "metis.lib ";
	maker.m_libs += maker.m_elib + "libmat.lib ";
	maker.m_libs += maker.m_elib + "libfea.lib ";
	maker.m_libs += maker.m_elib + "libamd.lib ";
	maker.m_libs += maker.m_elib + "libcamd.lib ";
	maker.m_libs += maker.m_elib + "libcolamd.lib ";
	maker.m_libs += maker.m_elib + "libccolamd.lib ";
	maker.m_libs += maker.m_elib + "libcholmod.lib ";
	maker.m_libs += maker.m_elib + "libumfpack.lib ";
	maker.m_libs += maker.m_elib + "libquadrule.lib ";
	maker.m_libs += maker.m_elib + "suitesparseconfig.lib ";
}
void setup_libs(Maker& maker)
{
	maker.m_libs.clear();
	maker.m_libs += "glu32.lib ";
	maker.m_libs += "opengl32.lib ";
	maker.m_libs += maker.m_elib + "gmsh.lib ";
	maker.m_libs += maker.m_elib + "libblas.lib ";
	maker.m_libs += maker.m_elib + "liblapack.lib ";
	maker.m_mode.compare("debug") == 0 ? setup_libs_debug(maker) : setup_libs_release(maker);
}

int main(int argc, char** argv)
{
	//setup
	Maker maker;
	maker.m_out = "ben";
	maker.setup(argc, argv);
	//build
	if(!maker.m_clean)
	{
		build_rsc();
		setup_libs(maker);
		setup_dlls(maker);
		maker.build_src();
		maker.build_dll();
		maker.build_exe();
		maker.build_run();
		maker.build_debug();
	}
	if(maker.m_clean)
	{
		maker.build_clean();
	}
	//return
	return EXIT_SUCCESS;
}