#pragma once

//std
#include <string>
#include <filesystem>

void print_file(std::string);
void join_logs(std::string = "");

std::filesystem::path uic_to_inc(std::filesystem::path);
std::filesystem::path inc_to_src(std::filesystem::path);
std::filesystem::path rsc_to_src(std::filesystem::path);
std::filesystem::path inc_to_obj(std::filesystem::path, std::string);
std::filesystem::path src_to_obj(std::filesystem::path, std::string);
std::filesystem::path rsc_to_obj(std::filesystem::path, std::string);
std::filesystem::path ext_to_dll(std::filesystem::path, std::string);

std::string& strip(std::string&, std::string);

std::string& replace(std::string&, std::string, std::string, unsigned = UINT_MAX);
std::filesystem::path& replace(std::filesystem::path&, std::string, std::string, unsigned = UINT_MAX);

void dep_list(std::filesystem::path, std::vector<std::filesystem::path>&, const std::vector<std::filesystem::path>&);