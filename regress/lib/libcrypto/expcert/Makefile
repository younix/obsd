# $OpenBSD$

LDFLAGS += -lcrypto

PROG = expcrt

PKG_REQUIRE != pkg_info -e 'p5-IO-Socket-SSL-*'
.if empty (PKG_REQUIRE)
regress:
	@echo "missing package p5-IO-Socket-SSL"
	@echo SKIPPED
.endif

REGRESS_TARGETS =	test-chain-with-root-CA
REGRESS_TARGETS +=	test-chain-without-root-CA
REGRESS_SETUP_ONCE =	create-certs

REGRESS_EXPECTED_FAILURES = test-chain-without-root-CA

create-certs: create-certs.pl ${PROG}
	perl ${.CURDIR}/create-certs.pl

test-chain-with-root-CA:
	./expcrt -e 10 -r

test-chain-without-root-CA:
	./expcrt -e 10

.include <bsd.regress.mk>
