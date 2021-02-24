/* $OpenBSD$ */
/*
 * Copyright (c) 2021 Jan Klemkow <j.klemkow@wemelug.de>
 * Copyright (c) 2021 Anton Borowka <aboro@genua.de>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <err.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/x509_vfy.h>

int error_found = 0;
int expected_error = 0;

static int
x509verify_error_handler(int ok, X509_STORE_CTX *ctx)
{
	char name[BUFSIZ] = { 0 };

	if (ctx->current_cert) {
		X509_NAME_oneline(X509_get_subject_name(ctx->current_cert),
		    name, sizeof name);
	}

	if (ok == 0)
		warnx("cert: %s error: %d %s", name, ctx->error,
		    X509_verify_cert_error_string(ctx->error));

	if (ctx->error == expected_error)
		error_found = 1;

	return 1;
}

X509_STORE *
store_init(const char *path)
{
	X509_STORE *store;
	X509_LOOKUP *lookup;

	/* init store object */
	if ((store = X509_STORE_new()) == NULL)
		return NULL;

	lookup = X509_STORE_add_lookup(store, X509_LOOKUP_file());
	if (X509_LOOKUP_load_file(lookup, path, X509_FILETYPE_PEM) == 0) {
		X509_STORE_free(store);
		return NULL;
	}

	X509_STORE_set_flags(store, X509_V_FLAG_PARTIAL_CHAIN);
	X509_STORE_set_verify_cb(store, x509verify_error_handler);

	ERR_clear_error();
	return store;
}

int
store_verify(X509_STORE *store, X509 *cert, STACK_OF(X509) *chain)
{
	X509_STORE_CTX *csc;
	int status = 0;

	/* prepare ctx:  */
	if ((csc = X509_STORE_CTX_new()) == NULL) {
		warnx("X509_STORE_CTX_new failed");
		return 1;
	}

	if (X509_STORE_CTX_init(csc, store, cert, chain) == 0) {
		X509_STORE_CTX_free(csc);
		warnx("X509_STORE_CTX_init failed");
		return 1;
	}

	X509_STORE_CTX_set_flags(csc, X509_V_FLAG_TRUSTED_FIRST);
#ifdef X509_V_FLAG_LEGACY_VERIFY
	X509_STORE_CTX_set_flags(csc, X509_V_FLAG_LEGACY_VERIFY);
#endif
	/* verify */
	status = X509_verify_cert(csc);

	if (error_found != 1) {
		warnx("expected error %d not found", expected_error);
		return 1;
	}

	return 0;
}

X509 *
cert_init(const char *path)
{
	FILE *fh;
	X509 *x509;

	if ((fh = fopen(path, "r")) == NULL)
		err(1, "fopen: %s", path);

	x509 = PEM_read_X509(fh, NULL, NULL, NULL);

	if (fclose(fh) == EOF)
		err(1, "fclose: %s", path);

	return x509;
}

STACK_OF(X509) *
chain_init(const char *path)
{
	FILE *fh;
	STACK_OF(X509_INFO) *sk;
	STACK_OF(X509) *st;
	X509_INFO *xi;

	if (path == NULL)
		return NULL;

	if ((fh = fopen(path, "r")) == NULL)
		err(1, "fopen: %s", path);

	if ((sk = PEM_X509_INFO_read(fh, NULL, NULL, NULL)) == NULL)
		err(1, "error reading chain info");

	if (fclose(fh) == EOF)
		err(1, "fclose: %s", path);

	if ((st = sk_X509_new_null()) == NULL) {
		sk_X509_INFO_pop_free(sk, X509_INFO_free);
		err(1, "reading chain memory error");
	}

	while (sk_X509_INFO_num(sk)) {
		xi = sk_X509_INFO_shift(sk);
		if (xi->x509 != NULL) {
			sk_X509_push(st, xi->x509);
			xi->x509=NULL;
		}
		X509_INFO_free(xi);
	}

	sk_X509_INFO_free(sk);

	if (sk_X509_num(st) == 0)
		printf("no chain certificates loaded\n");

	return st;
}

void
usage(void)
{
	err(1, "usage: expcrt [-r] [-e error]");
}

int
main(int argc, char *argv[])
{
	X509_STORE *store;
	X509 *cert;
	STACK_OF(X509) *chain;
	char *path_store = NULL;
	char *path_chain = NULL;
	const char *errstr;
	int root_flag = 0;
	int ch;

	while ((ch = getopt(argc, argv, "e:r")) != -1) {
		switch (ch) {
		case 'e':
			expected_error = strtonum(optarg, 0, INT_MAX, &errstr);
			if (errstr != NULL)
				errx(1, "%s: %s", errstr, optarg);
			break;
		case 'r':
			root_flag = 1;
			break;
		default:
			usage();
		}
	}

	if (root_flag == 1) {
		/*
		 * with root CA
		 */
		path_store = strdup("root.crt");
		path_chain = strdup("intermediate.crt");
	} else {
		/*
		 * without root CA
		 */
		path_store = strdup("intermediate.crt");
	}

	store = store_init(path_store);
	chain = chain_init(path_chain);
	cert = cert_init("expired.crt");

	return store_verify(store, cert, chain);
}
