using dependancy : 
https://github.com/marzer/tomlplusplus.git
which is cloned to opt locally on my mac - need to look where on mimir (ask Mike/Frantz)
also Eigen which should have system install 

compiling locally on my mac
clang++ -std=c++17 -I/opt/tomlplusplus/include -I/opt/homebrew/include/eigen3 rtc.cpp baldr.cpp -o rtc_test