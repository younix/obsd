#!/usr/bin/perl

# Copyright (c) 2021 Anton Borowka <aboro@genua.de>
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

use strict;
use warnings;

use IO::Socket::SSL::Utils;

my %certs;

@{$certs{root}}{qw/cert key/} = CERT_create(
    CA => 1,
    not_after => time() + 315360000,
    subject => { commonName => 'Root CA' },
);

@{$certs{intermediate}}{qw/cert key/} = CERT_create(
    CA => 1,
    issuer => [@{$certs{root}}{qw/cert key/}],
    not_after => time() + 315360000,
    subject => { commonName => 'Intermediate CA' },
);

@{$certs{expired}}{qw/cert key/} = CERT_create(
    issuer => [@{$certs{intermediate}}{qw/cert key/}],
    not_before => time() - 7200,
    not_after => time() - 3600,
    subject => { commonName => 'Expired' },
);

for (sort keys %certs) {
    PEM_cert2file($certs{$_}{cert}, "$_.crt");
}
