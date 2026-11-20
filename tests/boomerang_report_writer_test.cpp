#include "boomerang_report_writer.hpp"
#include <cassert>
#include <string>
using boomerang_trace::ReportWriter;
static std::string captured;
static bool sink(const char* p,unsigned n){captured.append(p,n);return true;}
static bool failed(const char*,unsigned){return false;}
int main() {
  ReportWriter w;w.reset(sink);
  w.print("BOOM_CSV,row_index,ms\n");
  w.print("BOOM_CSV,%u,",0u);w.print("%u\n",0u);
  w.print("BOOM_CSV,%u,%u\n",1u,30u);
  assert(w.ok&&w.emitted==2&&w.used==0);
  assert(captured=="BOOM_CSV,row_index,ms\nBOOM_CSV,0,0\nBOOM_CSV,1,30\n");
  uint32_t h=2166136261u;
  for(char c:std::string("BOOM_CSV,0,0\nBOOM_CSV,1,30\n")){h^=static_cast<unsigned char>(c);h*=16777619u;}
  assert(w.csv_hash==h);
  w.reset(failed);w.print("BOOM_CSV,0,0\n");assert(!w.ok&&w.emitted==0);
  w.reset(sink);w.print("%s",std::string(4096,'x').c_str());assert(!w.ok);
  w.reset(sink);w.print("\x01\n");assert(!w.ok);
  w.reset(sink);w.print("BOOM_CSV,0,nan\n");assert(w.ok&&w.emitted==1);
}
