"""Check the 180-degree mounting transform without changing field calibration."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
USER = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/USER'
compiler = os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
if not compiler:
    raise SystemExit('Set ROUTE_HOST_CC to a C11 compiler.')
source = r'''
#include <stdio.h>
#include "route_plan.h"
#include "route_config.h"
int main(void) {
    unsigned i; float x,y;
    printf("%u %u\n",ROUTE_PHYSICAL_FRONT_CENTER_MM,ROUTE_PHYSICAL_REAR_CENTER_MM);
    for(i=1;i<=17;i++) {
        const RouteSegment *s=RoutePlan_Get(i);
        const RouteAuxReference *a=RoutePlan_AuxReference(i);
        RoutePlan_Target(i,&x,&y);
        printf("%u %u %u %u %u %u %u %u %.0f %.0f %.0f %u %u %u %u\n",
            s->motion,s->trigger,s->side_wall,s->reference,s->distance_increases,
            s->mark.target_mm,s->mark.slow_band_mm,s->mark.turn_band_mm,
            s->length_mm,x,y,s->edge_to_center_mm,a?a->sensor:4,
            a?a->mark.target_mm:0,a?a->distance_increases:0);
    }
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='route-orientation-') as temp:
    build = Path(temp)
    for name in ('route_plan.c', 'route_plan.h', 'route_io.h'):
        shutil.copyfile(USER / name, build / name)
    (build / 'test.c').write_text(source, encoding='utf-8')
    config = (USER / 'route_config.h').read_text(encoding='utf-8')
    outputs = []
    for reverse in (0, 1):
        active = re.sub(r'(#define ROUTE_CHASSIS_REVERSED\s+)1',
                        lambda m: m[1] + str(reverse), config)
        (build / 'route_config.h').write_text(active, encoding='utf-8')
        exe = build / (f'test-{reverse}.exe' if os.name == 'nt' else f'test-{reverse}')
        subprocess.run([compiler, '-std=c11', '-I', str(build),
                        str(build / 'route_plan.c'), str(build / 'test.c'),
                        '-o', str(exe)], check=True)
        outputs.append([list(map(int, row.split())) for row in
                        subprocess.check_output([str(exe)], text=True).splitlines()])
    normal, reversed_ = outputs
    assert normal[0] == reversed_[0][::-1]
    for a, b in zip(normal[1:], reversed_[1:]):
        assert b[0] == (3, 2, 1, 0)[a[0]]
        for col in (1, 2, 12):
            assert b[col] == (1, 0, 3, 2, 4)[a[col]]
        for col in (3, 4, 5, 6, 7, 8, 11, 13, 14):
            assert b[col] == a[col]
        assert b[9] == -a[9] and b[10] == -a[10]
print('PASS: all 17 motions, primary/side/auxiliary lasers, targets and center references rotate consistently')
