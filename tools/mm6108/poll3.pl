# poll3.pl <secs> <out> [interval]
#   Ring poller for the relocated instrument. One perl process, one open fd.
#   Per poll it writes:  P <host_ns_before> <host_ns_after> <ctrl> <mtime> \n <2048B ring hex>
#   ctrl is the global write counter, so entry i of the dump is global index k with
#   k = ctrl-512 + ((i - ctrl) mod 512) -- that reconstructs the stream EXACTLY and makes ring
#   loss detectable (ctrl advancing by more than 512 between polls) instead of silent.
#   ⚠ poll at 0.2 s, not tight-loop: the poll shares the SPI bus AND the driver bus mutex with TX
#   and a tight loop MANUFACTURES the tail it is there to measure (sd 12.3 -> 105.8 us, MEASURED).
use Time::HiRes qw(clock_gettime CLOCK_REALTIME sleep);
my ($secs,$out,$iv)=($ARGV[0],$ARGV[1],$ARGV[2]//0.2);
open(my $fh,"+<","/dev/morse_io") or die "morse_io: $!";
open(my $o,">",$out) or die "$out: $!";
$|=1;
sub rd { my ($a,$n)=@_; ioctl($fh,0x6B01,$a) or die "ioctl: $!"; my $b=""; my $r=sysread($fh,$b,$n);
         die "short" unless defined($r) && $r==$n; return $b; }
my $t0=clock_gettime(CLOCK_REALTIME);
while (clock_gettime(CLOCK_REALTIME)-$t0 < $secs) {
  my $h0=clock_gettime(CLOCK_REALTIME);
  my $ctrl=unpack("V",rd(0x8020D0D8,4));
  my ($lo,$hi)=unpack("VV",rd(0x0200bff8,8));
  my $ring=""; $ring.=rd(0x8020DA80+$_*512,512) for (0..3);
  my $h1=clock_gettime(CLOCK_REALTIME);
  printf $o ("P %.0f %.0f %u %u\n%s\n",$h0*1e9,$h1*1e9,$ctrl,$hi*4294967296+$lo,unpack("H*",$ring));
  sleep($iv);
}
close $o;
