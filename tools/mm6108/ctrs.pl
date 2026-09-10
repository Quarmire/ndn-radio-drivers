# ctrs.pl -- read the relocated instrument block: ctrl + 4 site counters + MPE diag.
open(my $fh,"+<","/dev/morse_io") or die $!;
ioctl($fh,0x6B01,0x8020D0D8) or die $!;
my $b=""; sysread($fh,$b,32) or die "read";
my @w=unpack("V8",$b);
printf("ctrl=%u SUB=%u GO=%u IRQ=%u CNT=%u | mpe_calls=%u mpe_ptr=0x%08x mpe_busy=%u\n",@w[0..5],$w[6],$w[7]);
