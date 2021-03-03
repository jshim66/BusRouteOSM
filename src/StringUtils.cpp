#include "../include/StringUtils.h"
#include <algorithm>
#include <iostream>
#include <cctype>
#include <cstdio>
#include <cmath>


namespace StringUtils {

    std::string Slice(const std::string &str, ssize_t start, ssize_t end) {
        if (start < 0) {
            start += str.size();
        }
        if (end < 0) {
            end = str.size() + end;
            end -= start;
        } else {
            end -= start;
        }
        if (start > str.size()) {
            return "";
        }
        return str.substr(start, end);
    }

    std::string Capitalize(const std::string &str) {
        std::string x;
        for (char c: str) {
            x += tolower(c);
        }
        x[0] = toupper(str[0]);
        return x;
    }

    std::string Title(const std::string &str) {
        std::string y;
        for (char c: str) {
            y += tolower(c);
        }
        if (str[0] != ' ') {
            y[0] = toupper(str[0]);
        }
        for (int i = 1; i <= str.size(); i++) {
            if ((str[i - 1] == ' ') || ispunct(str[i - 1]) || isdigit(str[i - 1])) {
                y[i] = toupper(str[i]);
            }
        }
        return y;
    }

    std::string LStrip(const std::string &str) {
        // http://www.cplusplus.com/reference/string/basic_string/find_first_not_of/
        int start = str.find_first_not_of(' ');
        return str.substr(start);
    }

    std::string RStrip(const std::string &str) {
        // http://www.cplusplus.com/reference/string/basic_string/find_last_not_of/
        int end = str.find_last_not_of(' ');
        end = end + 1;
        return str.substr(0, end);
    }

    std::string Strip(const std::string &str) {
        int start = str.find_first_not_of(' ');
        int end = str.find_last_not_of(' ');
        end = end - start + 1;
        return str.substr(start, end);
    }

    std::string Center(const std::string &str, int width, char fill) {
        int spaces = width - str.size();
        int lSpaces = spaces / 2;
        int rSpaces = spaces - lSpaces;
        std::string temp;
        for (int i = 0; i < lSpaces; i++) {
            temp += fill;
        }
        for (char c: str) {
            temp += c;
        }
        for (int i = 0; i < rSpaces; i++) {
            temp += fill;
        }

        return temp;
    }

    std::string LJust(const std::string &str, int width, char fill) {
        std::string temp;
        for (char c: str) {
            temp += c;
        }
        for (int i = 0; i < width - str.size(); i++) {
            temp += fill;
        }
        return temp;

    }

    std::string RJust(const std::string &str, int width, char fill) {
        std::string temp;
        for (int i = 0; i < width - str.size(); i++) {
            temp += fill;
        }
        for (char c: str) {
            temp += c;
        }
        return temp;

    }

    std::string Replace(const std::string &str, const std::string &old, const std::string &rep) {
        std::string temp;
        for (int i = 0; i < str.size(); i++) {
            if (str.substr(i, old.size()) == old) {
                temp += rep;
                i += old.size() - 1;
            } else {
                temp += str[i];
            }
        }
        return temp;

    }

    std::vector<std::string> Split(const std::string &str, const std::string &splt) {
        std::vector<std::string> v;
        std::string temp;
        std::string temp1;
        if (splt == "") {
            temp = StringUtils::Replace(str, "\t", " ");
            temp = StringUtils::Replace(temp, "\n", " ");

            for (int j = 0; j < temp.size(); ++j) {
                int count = 0;
                while (temp[j + count] == ' ') {
                    count++;
                }
                temp1 += temp[j];
                if (count != 0) {
                    j += count - 1;
                } else {
                    j += count;
                }
            }
            return StringUtils::Split(temp1, " ");

        } else {
            for (int i = 0; i < str.size(); i++) {
                if (str.substr(i, splt.size()) != splt) {
                    temp += str[i];
                } else {
                    v.push_back(temp);
                    temp = "";
                }
            }
            v.push_back(temp);
        }

        return v;

    }

    std::string Join(const std::string &str, const std::vector<std::string> &vect) {
        std::string temp;
        temp = "";
        int count = 0;
        for (auto c: vect) {
            temp += c;
            if (count == vect.size() - 1) {
                break;
            }
            temp += str;
            count++;
        }
        return temp;
    }

    std::string ExpandTabs(const std::string &str, int tabsize) {
        std::vector<std::string> vec = StringUtils::Split(str, "\t");
        std::vector<std::string> new_vec;
        std::string temp;
        int index = 0;
        for (auto s: vec) {
            temp += s;
            if ((index != (vec.size() - 1)) && (tabsize != 0)) {
                int x = tabsize - s.size();
                if (x < 0) {
                    for (int i = 0; i < s.size() - tabsize + 1; i++) {
                        temp += ' ';
                    }
                } else if (x > 0) {
                    for (int i = 0; i < tabsize - s.size(); i++) {
                        temp += ' ';
                    }
                } else {
                        for (int i = 0; i < tabsize; i++) {
                            temp += ' ';
                        }
                }
            }
            new_vec.push_back(temp);
            temp = "";
            index++;
            }
        return StringUtils::Join("", new_vec);
    }


    int EditDistance(const std::string &left, const std::string &right, bool ignorecase) {
//        if (ignorecase == true) {
//            return 0;
//        }
//        int difference = 0;
//        int size_difference = left.size() - right.size();
//        int len = abs(size_difference);
//        int index = 0;
//        for (char c: left) {
//            if (c != right[index]) {
//                difference += 1;
//            }
//            index++;
//        }
//        if (len != 0) {
//            difference += len;
//        }
//
//        return difference;

        // https://en.wikibooks.org/wiki/Algorithm_Implementation/Strings/Levenshtein_distance#Python
            if (ignorecase == true) {
                return 0;
            }

            const std::size_t len1 = left.size(), len2 = right.size();
            std::vector<std::vector<unsigned int>> d(len1 + 1, std::vector<unsigned int>(len2 + 1));

            d[0][0] = 0;
            for(unsigned int i = 1; i <= len1; ++i) d[i][0] = i;
            for(unsigned int i = 1; i <= len2; ++i) d[0][i] = i;

            for(unsigned int i = 1; i <= len1; ++i)
                for(unsigned int j = 1; j <= len2; ++j)
                    d[i][j] = std::min({ d[i - 1][j] + 1, d[i][j - 1] + 1, d[i - 1][j - 1] + (left[i - 1] == right[j - 1] ? 0 : 1) });
            return d[len1][len2];
        }


}
