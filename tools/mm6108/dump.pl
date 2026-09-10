# dump.pl <start_hex> <total_bytes> [chunk]
my $start = hex($ARGV[0]); my $total = $ARGV[1]; my $chunk = $ARGV[2] // 256;
open(my $fh, "+<", "/dev/morse_io") or die "open: $!";
my $out = "";
for (my $o = 0; $o < $total; $o += $chunk) {
    ioctl($fh, 0x6B01, $start + $o) or die "ioctl: $!";
    my $buf = ""; my $n = sysread($fh, $buf, $chunk);
    die "short read at $o" unless defined($n);
    $out .= $buf;
}
print unpack("H*", $out), "\n";
