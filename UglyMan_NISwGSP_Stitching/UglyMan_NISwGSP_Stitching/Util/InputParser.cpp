//
//  InputParser.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "InputParser.h"

class StrTok {
public:
    StrTok(const string & str_input,
           const string & str_delim = " ,") {
        string::size_type nHead, nTail;
        nHead = str_input.find_first_not_of(str_delim, 0);
        nTail = str_input.find_first_of(str_delim, nHead);
        
        while(nHead != string::npos || nTail != string::npos) {
            tokened.emplace_back(str_input.substr(nHead, nTail - nHead));
            nHead = str_input.find_first_not_of(str_delim, nTail);
            nTail = str_input.find_first_of(str_delim, nHead);
        }
    }
    std::vector<std::string> tokened;
};

InputParser::InputParser(const string & file_name) {
    ifstream file(file_name);
    string line;
    
    while(getline(file, line)) {
        StrTok st(line, " {|}\n\r\t");
        data[st.tokened[0]] = st.tokened[1];
    }
    file.close();
}

template <typename T>
T transfer(const string & str) {
    T result;
    try {
        if(std::is_same<T, int>::value) {
            result = stoi(str);
        } else if(std::is_same<T, float>::value) {
            result = stof(str);
        } else if(std::is_same<T, double>::value) {
            result = stod(str);
        } else {
            printError("F(transfer) transfer function");
        }
    } catch (const invalid_argument & inv_argument) {
        printError("F(transfer) invalid argument: " + string(inv_argument.what()));
    }
    return result;
}

template<>
string transfer<string>(const string & str) {
    return str;
}

template <typename T>
T InputParser::get(const string & key, const T * ptr) const {
    T result;
    if(data.find(key) == data.end()) {
        if(ptr == NULL) {
            printError("F(get) key error: " + key);
        } else {
            result = *ptr;
        }
    } else {
        const string value = data.at(key);
        result = transfer<T>(value);
    }
    return result;
}

template <typename T>
vector<T> InputParser::getVec(const string & key,
                              const bool sure_exist) const {
    vector<T> result;
    if(data.find(key) == data.end()) {
        if(sure_exist) {
            printError("F(getVec) key error: " + key);
        }
    } else {
        const string value = data.at(key);
        StrTok st(value, " (,)\n\r\t");
        result.reserve(st.tokened.size());
        for(int i = 0; i < st.tokened.size(); ++i) {
            result.emplace_back(transfer<T>(st.tokened[i]));
        }
    }
    return result;
}

template    int InputParser::get<   int>(const string & key, const    int * ptr) const;
template  float InputParser::get< float>(const string & key, const  float * ptr) const;
template double InputParser::get<double>(const string & key, const double * ptr) const;
template string InputParser::get<string>(const string & key, const string * ptr) const;

template vector<   int> InputParser::getVec<   int>(const string & key, const bool sure_exist) const;
template vector< float> InputParser::getVec< float>(const string & key, const bool sure_exist) const;
template vector<double> InputParser::getVec<double>(const string & key, const bool sure_exist) const;
template vector<string> InputParser::getVec<string>(const string & key, const bool sure_exist) const;

