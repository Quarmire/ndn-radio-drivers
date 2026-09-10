my $start = hex($ARGV[0]); my $total = $ARGV[1]; my $chunk = 256;
open(my $fh, "+<", "/dev/morse_io") or die "open: $!";
for (my $o = 0; $o < $total; $o += $chunk) {
    ioctl($fh, 0x6B01, $start + $o) or die "ioctl: $!";
    syswrite($fh, "\0" x $chunk, $chunk) or die "wr: $!";
}
print "zeroed $total bytes at $ARGV[0]\n";
