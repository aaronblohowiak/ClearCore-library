#include <cstdio>
#include "Cutter.h"
#include "test/TestSerial.h"

using namespace Cutter;

int main() {
    TestSerial serial;
    Controller cutter(&serial);

    const char* cmds[] = {
        "ping",
        "version",
        "status",
        "ping seq=5",
        "ping seq=3",         // stale seq
        "configure_stepper motor=0",
        "move motor=0 steps=1000",
        "bogus_command",
    };
    for (const char* c : cmds) {
        serial.ClearOutput();
        serial.SendLine(c);
        cutter.Update();
        printf("> %-32s | %s", c, serial.GetOutput().c_str());
        if (serial.GetOutput().empty()) printf("  <NO RESPONSE>\n");
    }
    return 0;
}
