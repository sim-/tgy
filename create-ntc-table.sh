#! /bin/sh

RPULLUP_VALUES="`awk -F= '/^\.equ[ \t]*mux_temperature_rpullup/{C[$2]++}END{for(i in C){if(i!=0)printf "%i ",i;}}' *.inc`"
RNTC_VALUES="`awk -F= '/^\.equ[ \t]*mux_temperature_rntc[ \t]*=/{C[$2]++}END{for(i in C){if(i!=0)printf "%i ",i;}}' *.inc`"
BETA_VALUES="`awk -F= '/^\.equ[ \t]*mux_temperature_beta[ \t]*=/{C[$2]++}END{for(i in C){if(i!=0)printf "%i ",i;}}' *.inc`"

test -z "$1" || RPULLUP_VALUES=$1
test -z "$2" || RNTC_VALUES=$2
test -z "$3" || BETA_VALUES=$3

echo "; NTC thermal tables"
echo "; generated via script: $0"
echo ".if defined(mux_temperature_rpullup) && defined(mux_temperature_rntc) && defined(mux_temperature_beta)"
(
for R in $RPULLUP_VALUES; do
 for T in $RNTC_VALUES; do
  for B in $BETA_VALUES; do
    echo "$R $T $B";
  done
 done
done
) | awk '
function printline(A,B) {
  # scale and saturate
  C=B*256;
  if (C<-32768){ C=-32768; }
  if (C>32767){ C=32767; }
  TS=C/256;
  if (C<0){C+=65536};
  # and print
  printf "\t.dw 0x%04x\t; ADC = %4i, Temp = %6.1fC\n", C, A, TS;
}
{
  RPULLUP=$1;
  RTHERMAL=$2;
  BETA=$3;
  printf ";----------------------------------------------------------------\n";
  printf "; RPULLUP = %i, RTHERMAL = %i, BETA = %i\n", RPULLUP, RTHERMAL, BETA;
  printf ".%sif (mux_temperature_rpullup == %i) && (mux_temperature_rntc == %i) && (mux_temperature_beta == %i)\n", PREFIX, RPULLUP, RTHERMAL, BETA;
  PREFIX="else";
  printline(0, -100*RPULLUP);
  for(ADC=1;ADC<255;ADC++){
    # handle both pullups/pulldowns
    if (RPULLUP<0) {
      X=-((255/(255-ADC))-1)*RPULLUP/RTHERMAL;
    } else {
      X=((255/ADC)-1)*RPULLUP/RTHERMAL;
    }
    # do the real calculation
    ITK=log(X)/BETA + 1/(273.15+25);
    T=1/ITK -273.15;
  printline(ADC,T)
  }
  printline(255, 100*RPULLUP);
}
END{if (NR>0) {printf ".else\n\t.error \"No corresponding BETA, RPULLUP and RNTC values found - please recreate ntc-table.inc via make to reflect your changes to .inc\"\n.endif\n.endif\n";}}'
