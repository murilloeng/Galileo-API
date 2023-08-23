cl /nologo /std:c++20 /EHsc /c /openmp /D "NDEBUG" /O2 /MD win/make.cpp /Fo:win/make.obj
cl /nologo /std:c++20 /EHsc /c /openmp /D "NDEBUG" /O2 /MD ../win/Maker.cpp /Fo:../win/Maker.obj
link /nologo /out:make.exe win/make.obj ../win/Maker.obj