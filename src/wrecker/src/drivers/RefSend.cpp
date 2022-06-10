//#pragma once

#include "Send.h"

using namespace std;

void RefSend::setHeader(Header* header){
    this->header = header;
}
void RefSend::setContent(Content* content){
    this->content = content;
}
void RefSend::setFooter(Footer* footer){
    this->footer = footer;
}
Header* RefSend::getHeader(){
    return this->header;
}
Content* RefSend::getContent(){
    return this->content;
}
Footer* RefSend::getFooter(){
    return this->footer;
}
