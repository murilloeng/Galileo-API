cl /nologo /std:c++20 /EHsc /c /openmp /D "NDEBUG" /O2 /MD maker/make.cpp /Fo:maker/make.obj
cl /nologo /std:c++20 /EHsc /c /openmp /D "NDEBUG" /O2 /MD maker/path.cpp /Fo:maker/path.obj
link /nologo /out:make.exe maker/make.obj maker/path.obj