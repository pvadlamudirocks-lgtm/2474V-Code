#pragma once
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdint>
namespace boomerang_trace {
// Report-only bounded line assembler. Sink performs paced I/O after motion.
struct ReportWriter {
  using Sink=bool (*)(const char*,unsigned);
  char line[4096]{},fragment[4096]{};
  unsigned used=0,emitted=0;
  uint32_t csv_hash=2166136261u;
  bool ok=true; const char* failure="NONE";
  Sink sink=nullptr;
  void reset(Sink output) {used=emitted=0;csv_hash=2166136261u;ok=true;failure="NONE";sink=output;}
  void print(const char* fmt,...) __attribute__((format(printf,2,3))) {
    if(!ok)return;
    va_list args;va_start(args,fmt);
    const int n=std::vsnprintf(fragment,sizeof(fragment),fmt,args);va_end(args);
    if(n<0||static_cast<unsigned>(n)>=sizeof(fragment)){ok=false;failure="FORMAT_OR_LINE_OVERFLOW";return;}
    for(int i=0;i<n&&ok;++i) {
      if(used>=sizeof(line)-1){ok=false;failure="FORMAT_OR_LINE_OVERFLOW";return;}
      const char ch=fragment[i];
      if((static_cast<unsigned char>(ch)<32&&ch!='\n')||
         static_cast<unsigned char>(ch)>126){ok=false;failure="NON_ASCII_CONTROL_BYTE";return;}
      line[used++]=ch;
      if(ch=='\n') {
        line[used]=0;
        const bool row=std::strncmp(line,"BOOM_CSV,",9)==0&&line[9]>='0'&&line[9]<='9';
        if(!sink||!sink(line,used)){ok=false;failure="OUTPUT_OR_MODE_INTERRUPTION";return;}
        if(row) {
          for(unsigned j=0;j<used;++j){csv_hash^=static_cast<unsigned char>(line[j]);csv_hash*=16777619u;}
          ++emitted;
        }
        used=0;
      }
    }
  }
};
}
