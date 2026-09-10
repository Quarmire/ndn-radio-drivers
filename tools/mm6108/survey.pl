# survey.pl <start_hex> <end_hex> -- dump [start,end) as "addr hex" lines of 64 B.
# ⚠ NEVER give an address outside mapped chip RAM: one out-of-range dm_read makes
# morse_spi_cmd53_read fail -71 and every later SPI access read 0xffffffff, which
# needs an unbind/bind to clear. Measured top of the mapped MAC window: >=0x8020C900,
# <0x80210000. Stay well inside it.
my ($s,$e)=(hex($ARGV[0]),hex($ARGV[1]));
open(my $fh,"+<","/dev/morse_io") or die "open: $!";
for (my $a=$s;$a<$e;$a+=256){
  ioctl($fh,0x6B01,$a) or die "ioctl: $!";
  my $b=""; my $n=sysread($fh,$b,256);
  die "short read at ".sprintf("0x%08x",$a) unless defined($n) && $n==256;
  for (my $o=0;$o<256;$o+=64){ printf("%08x %s\n",$a+$o,unpack("H*",substr($b,$o,64))); }
}
