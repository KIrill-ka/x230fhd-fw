set -e
DRV=stm8s-sdcc
CFLAGS="--std-sdcc99 --opt-code-size -mstm8 -DSTM8S003 -I $DRV/inc"
LDFLAGS="-lstm8 -mstm8 --out-fmt-ihx"
sdcc -mstm8 $CFLAGS -I $DRV -c main.c -o main.rel
sdcc -mstm8 $CFLAGS -I $DRV -c stm8s_it.c -o stm8s_it.rel
OBJ="main.rel stm8s_it.rel"
for i in stm8s_clk stm8s_tim1 stm8s_tim2 stm8s_adc1; do
	sdcc -mstm8 $CFLAGS $DRV/src/$i.c -c -o $i.rel
    OBJ="$OBJ $i.rel"
done
sdcc $LDFLAGS -o x230fhd-fw.hex $OBJ
packihx x230fhd-fw.hex > x230fhd-fw.hex.tmp
mv x230fhd-fw.hex.tmp x230fhd-fw.hex
