import os
import sys
import subprocess

def project_v1(name):        
    os.mkdir("doc")
    os.mkdir("include")
    os.mkdir("lib")
    os.mkdir("lib/MyLib")
    os.mkdir("examples")
    os.mkdir("data")
    os.mkdir("build")
    os.mkdir("cmd")
    
    tmp = "lib/MyLib"
    
    with open(tmp + '/MyLib.h', 'w') as file:
        file.write('')
        file.close()
    with open(tmp + '/MyLib.cpp', 'w') as file:
        file.write('')    
        file.close()
    with open(tmp + '/CmakeLists.txt', 'w') as file:
        tmp = """
add_library(MyLib STATIC MyLib.cpp)
target_include_directories(MyLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
        """
        file.write(tmp)
        file.close()
        
    with open('lib/CmakeLists.txt', 'w') as file:
        
        file.write('#add_subdirectory(MyLib)\n')
        file.close()
    
    os.mkdir("src")
    with open('src/main.cpp', 'w') as file:
        tmp = """
#include <stdio.h>
int main()
{
    printf("Hello World");
    return 0;
}

        """
        file.write(tmp)
        file.close()
        
    with open('CmakeLists.txt','w') as file:
        
        tmp = f"""
cmake_minimum_required(VERSION 3.14)
project({name})

"""
        
        tmp += """        
        
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

if(MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    set(CMAKE_CX_FLAGS "${CMAKE_C_FLAGS} -m32")
endif()

add_subdirectory(lib)

add_executable(app
src/main.cpp
)

target_link_libraries(app
PRIVATE
)

        """
                                
        
        file.write(tmp)
        file.close()
        
    with open("README.md", 'w') as file:
        file.write('')
        file.close()
        
    with open("cmd/init.bat", 'w') as file:
        tmp = 'cmake -S ../ -B ../build  -G "MinGW Makefiles"'
        file.write(tmp)
        file.close()
    
    with open("cmd/build.bat", 'w') as file:
        tmp = 'del /f /s /q "../build/app.exe" && cmake --build ../build --parallel 4 '
        file.write(tmp)
        file.close()
        
    with open("cmd/clean.bat", 'w') as file:
        tmp = 'rmdir /q /s "../build"'
        file.write(tmp)
        file.close()
        
    with open("cmd/run.bat", 'w') as file:
        tmp = '"../build/app.exe"'
        file.write(tmp)
        file.close()
        
    with open("cmd/build_run.bat", 'w') as file:
        tmp = 'build.bat && run.bat'
        file.write(tmp)
        file.close()
        
def lib_v1(name):
        os.mkdir("lib/"+name)
        
        #Create Headerfile
        with open("lib/"+name+"/"+name+".h", 'w') as file:
            tmp = f"""
#ifndef __LIB__{name.upper()}__H
#define __LIB__{name.upper()}__H

class {name} {{
public:             
    {name}();
    ~{name}();
private:
}}; 
#endif // __LIB__{name.upper()}__H
            """
            file.write(tmp)
            file.close()

        #Create Cppfile
        with open("lib/"+name+"/"+name+".cpp", 'w') as file:
            tmp = f"""
#include "{name}.h"
    
            """
            file.write(tmp)
            file.close()
            
        #Create lib/xxxx/CmakeLists.txt
        with open("lib/"+name+"/CmakeLists.txt", 'a') as file:
            tmp = "add_library("+name+" STATIC "+name+".cpp)\n"
            tmp+= "target_include_directories("+name+" PUBLIC $"+chr(123)+"CMAKE_CURRENT_SOURCE_DIR"+chr(125)+")"       
            
            file.write(tmp)
            file.close()
            
        with open("lib/CmakeLists.txt", 'a') as file:
            tmp ="add_subdirectory("+name+")\n"
            
            file.write(tmp)
            file.close()

mode=1
if __name__ == "__main__":
    if mode==1:
        if len(sys.argv) > 2:
            if(sys.argv[1] == 'project'):
                project_v1(sys.argv[2])
                
            elif(sys.argv[1] == 'lib'):
                print("Generating library:", sys.argv[2])
                lib_v1(sys.argv[2])
        else:
            print("error : error no project name.")
            
    elif mode==2:
        print("Mode : Test Script\n")
        with open("CmakeLists.txt", 'r') as file:
            r = file.readlines()
            #file.write(tmp)
            for line in r:
                
                if line.find("PRIVATE") != -1:
                    print("Found MyLib in CMakeLists.txt")
            file.close()
    

