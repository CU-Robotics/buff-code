#pragma once

#include <bitset>
#include <string>
#include <vector>
#include <math.h>

using namespace std;

string to_binaryString(int num, int length){
    if (to_string(num).length() > length){
        return "error";
    }
    bitset<sizeof(num)> bits(num);
    string result = bits.to_string();
    while (result.length() < length){
        result = "0" + result;
    }
    return result;
}

vector<uint8_t> to_bytes(string str){
    vector<uint8_t> bytes = vector<uint8_t>();
    if (str.length()%8 != 0){
        return bytes;
    }
    for (int _ = 0; _ < str.length()/8; _ ++){
        int num = 0;
        const string sub = str.substr(_*8, 8);
        for (int __ = 0; __ < 8; __++){
            num += (stoi(to_string(sub.at(__)))-48)*pow(2,7-__);
        }
        bytes.push_back(uint8_t(num));
    }
    return bytes;
}
