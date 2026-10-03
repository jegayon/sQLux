/*
 * validate_ops.c - Run the Tom Harte ProcessorTests for the 68000
 * (SingleStepTests/680x0, directory 68000/v1) on the sQLux CPU core and
 * compare every register, every flag and every byte of memory.
 *
 * Build (from the sQLux build directory):
 *     cmake --build . --target validate_ops
 * Run:
 *     ./validate_ops path/to/68000/v1/NAME.json.gz ...  (one or more files)
 *     ./validate_ops -v 5 ADD.b.json.gz     (show up to 5 failing tests)
 *     ./validate_ops -a -v 5 MULU.json.gz   (show failing address errors)
 *
 * Each test gives the state before and after one instruction. The
 * instruction starts at the initial "pc", and its first two words are in
 * "prefetch" (the rest, if any, in "ram"); the final "pc" is the address of
 * the next instruction. The core runs the instruction with a flat 16 MB of
 * RAM (no QL hardware, no ROM protection) and the result is compared field
 * by field.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#define SDL_MAIN_HANDLED        /* keep our own main() on Windows */
#include <SDL.h>

#include "QL68000.h"
#include "general.h"
#include "QL_screen.h"
#include "zx8301.h"
#include "qsound.h"

#ifndef SDLCALL
#define SDLCALL
#endif

void ExceptionProcessing(void);         /* iexl_general.c */
int EmulatorTable(void);                /* Init.c */
extern Cond doTrace;
extern rw16 saved_sr;                   /* iexl_general.c: SR before the instruction */

/* ------------------------------------------------------------------ */
/* The rest of the emulator: flat memory, a plain 68000                */
/* ------------------------------------------------------------------ */

#define MEM_SIZE 0x1000000

rw8 ReadHWByte(aw32 a) { return ((uint8_t *)memBase)[a & 0xffffff]; }
rw16 ReadHWWord(aw32 a) { return (rw16)RW((Ptr)memBase + (a & 0xffffff)); }
rw32 ReadHWLong(aw32 a) { return (rw32)RL((Ptr)memBase + (a & 0xffffff)); }
void WriteHWByte(aw32 a, aw8 d) { ((uint8_t *)memBase)[a & 0xffffff] = (uint8_t)d; }
void WriteHWWord(aw32 a, aw16 d) { WW((Ptr)memBase + (a & 0xffffff), d); }

/* TRAP #0-#3 are QDOS calls in sQLux: here they are plain traps */
static void plain_trap(void)
{
	exception = 32 + (code & 15);
	extraFlag = true;
	nInst2 = nInst;
	nInst = 0;
}
void trap0(void) { plain_trap(); }
void trap1(void) { plain_trap(); }
void trap2(void) { plain_trap(); }
void trap3(void) { plain_trap(); }

void DbgInfo(void) { }
void QLSDLScreenWrite(uint32_t addr, unsigned n) { (void)addr; (void)n; }
void QLSDLSnapshotLine(void) { }
uint64_t ql_snapshot_at = UINT64_MAX;
uint32_t snap_hot_lo = 0, snap_hot_hi = 0;
screen_specs qlscreen;
SDL_atomic_t doPoll;
int SDLCALL SDL_AtomicGet(SDL_atomic_t *a) { (void)a; return 0; }
void dosignal(void) { }
void ipc_lle_sync(void) { }
void mdv_sync(void) { }
int zx_contention = 0;
void zx8301_insn(uint64_t t0, uint32_t pc_addr, unsigned prog_words)
{ (void)t0; (void)pc_addr; (void)prog_words; }
void zx8301_ram(unsigned bytes, int is_write) { (void)bytes; (void)is_write; }
void qsound_write_byte(uint32_t addr, uint8_t val) { (void)addr; (void)val; }
uint8_t qsound_read_byte(uint32_t addr) { (void)addr; return 0; }
int speed = 20;
int tracetrap = 0;
int verbose = 0;
unsigned hw_trace_irq_taken[8];
void ql_irq_accepted(int level) { (void)level; }
void hw_trace_sample(void) { }

/* The addressing mode tables, as in general.c (general.c itself cannot be
 * linked: it also holds the QL hardware) */
rw32 GetEA_m2(ashort) AREGP;
rw32 GetEA_m5(ashort) AREGP;
rw32 GetEA_m6(ashort) AREGP;
rw32 GetEA_m7(ashort) AREGP;
rw32 GetEA_mBad(ashort) AREGP;
rw32 (*GetEA[8])(ashort) /*AREGP*/ = { GetEA_mBad, GetEA_mBad, GetEA_m2,
				       GetEA_mBad, GetEA_mBad, GetEA_m5,
				       GetEA_m6,   GetEA_m7 };

rw8 GetFromEA_b_m0(void);
rw8 GetFromEA_b_mBad(void);
rw8 GetFromEA_b_m2(void);
rw8 GetFromEA_b_m3(void);
rw8 GetFromEA_b_m4(void);
rw8 GetFromEA_b_m5(void);
rw8 GetFromEA_b_m6(void);
rw8 GetFromEA_b_m7(void);
rw8 (*GetFromEA_b[8])(void) = { GetFromEA_b_m0, GetFromEA_b_mBad,
				GetFromEA_b_m2, GetFromEA_b_m3,
				GetFromEA_b_m4, GetFromEA_b_m5,
				GetFromEA_b_m6, GetFromEA_b_m7 };

rw16 GetFromEA_w_m0(void);
rw16 GetFromEA_w_m1(void);
rw16 GetFromEA_w_m2(void);
rw16 GetFromEA_w_m3(void);
rw16 GetFromEA_w_m4(void);
rw16 GetFromEA_w_m5(void);
rw16 GetFromEA_w_m6(void);
rw16 GetFromEA_w_m7(void);
rw16 (*GetFromEA_w[8])(void) = { GetFromEA_w_m0, GetFromEA_w_m1, GetFromEA_w_m2,
				 GetFromEA_w_m3, GetFromEA_w_m4, GetFromEA_w_m5,
				 GetFromEA_w_m6, GetFromEA_w_m7 };

rw32 GetFromEA_l_m0(void);
rw32 GetFromEA_l_m1(void);
rw32 GetFromEA_l_m2(void);
rw32 GetFromEA_l_m3(void);
rw32 GetFromEA_l_m4(void);
rw32 GetFromEA_l_m5(void);
rw32 GetFromEA_l_m6(void);
rw32 GetFromEA_l_m7(void);
rw32 (*GetFromEA_l[8])(void) = { GetFromEA_l_m0, GetFromEA_l_m1, GetFromEA_l_m2,
				 GetFromEA_l_m3, GetFromEA_l_m4, GetFromEA_l_m5,
				 GetFromEA_l_m6, GetFromEA_l_m7 };

void PutToEA_b_m0(ashort, aw8) AREGP;
void PutToEA_b_mBad(ashort, aw8) AREGP;
void PutToEA_b_m2(ashort, aw8) AREGP;
void PutToEA_b_m3(ashort, aw8) AREGP;
void PutToEA_b_m4(ashort, aw8) AREGP;
void PutToEA_b_m5(ashort, aw8) AREGP;
void PutToEA_b_m6(ashort, aw8) AREGP;
void PutToEA_b_m7(ashort, aw8) AREGP;
void (*PutToEA_b[8])(ashort, aw8) /*REGP2*/ = { PutToEA_b_m0, PutToEA_b_mBad,
						PutToEA_b_m2, PutToEA_b_m3,
						PutToEA_b_m4, PutToEA_b_m5,
						PutToEA_b_m6, PutToEA_b_m7 };

void PutToEA_w_m0(ashort, aw16) AREGP;
void PutToEA_w_m1(ashort, aw16) AREGP;
void PutToEA_w_m2(ashort, aw16) AREGP;
void PutToEA_w_m3(ashort, aw16) AREGP;
void PutToEA_w_m4(ashort, aw16) AREGP;
void PutToEA_w_m5(ashort, aw16) AREGP;
void PutToEA_w_m6(ashort, aw16) AREGP;
void PutToEA_w_m7(ashort, aw16) AREGP;
void (*PutToEA_w[8])(ashort, aw16) /*REGP2*/ = { PutToEA_w_m0, PutToEA_w_m1,
						 PutToEA_w_m2, PutToEA_w_m3,
						 PutToEA_w_m4, PutToEA_w_m5,
						 PutToEA_w_m6, PutToEA_w_m7 };

void PutToEA_l_m0(ashort, aw32) AREGP;
void PutToEA_l_m1(ashort, aw32) AREGP;
void PutToEA_l_m2(ashort, aw32) AREGP;
void PutToEA_l_m3(ashort, aw32) AREGP;
void PutToEA_l_m4(ashort, aw32) AREGP;
void PutToEA_l_m5(ashort, aw32) AREGP;
void PutToEA_l_m6(ashort, aw32) AREGP;
void PutToEA_l_m7(ashort, aw32) AREGP;
void (*PutToEA_l[8])(ashort, aw32) /*REGP2*/ = { PutToEA_l_m0, PutToEA_l_m1,
						 PutToEA_l_m2, PutToEA_l_m3,
						 PutToEA_l_m4, PutToEA_l_m5,
						 PutToEA_l_m6, PutToEA_l_m7 };

/* ------------------------------------------------------------------ */
/* A small JSON reader, enough for this format                         */
/* ------------------------------------------------------------------ */

typedef struct jv {
	int type;               /* 'n' number, 's' string, 'a' array, 'o' object */
	double num;
	char *str;              /* string value, or key inside an object */
	struct jv **items;      /* array items or object values */
	char **keys;
	int n, cap;
} jv;

static const char *jp;

static void skip_ws(void) { while (*jp == ' ' || *jp == '\n' || *jp == '\r' || *jp == '\t') jp++; }

static jv *jnew(int type)
{
	jv *v = calloc(1, sizeof(jv));
	v->type = type;
	return v;
}

static void jpush(jv *v, char *key, jv *item)
{
	if (v->n == v->cap) {
		v->cap = v->cap ? v->cap * 2 : 8;
		v->items = realloc(v->items, v->cap * sizeof(jv *));
		v->keys = realloc(v->keys, v->cap * sizeof(char *));
	}
	v->keys[v->n] = key;
	v->items[v->n++] = item;
}

static char *jstring(void)
{
	const char *s = ++jp;
	while (*jp && *jp != '"') {
		if (*jp == '\\' && jp[1]) jp++;
		jp++;
	}
	size_t len = (size_t)(jp - s);
	char *r = malloc(len + 1);
	memcpy(r, s, len);
	r[len] = 0;
	if (*jp == '"') jp++;
	return r;
}

static jv *jparse(void)
{
	skip_ws();
	if (*jp == '{') {
		jv *o = jnew('o');
		jp++;
		for (;;) {
			skip_ws();
			if (*jp == '}') { jp++; break; }
			char *k = jstring();
			skip_ws();
			if (*jp == ':') jp++;
			jpush(o, k, jparse());
			skip_ws();
			if (*jp == ',') jp++;
		}
		return o;
	}
	if (*jp == '[') {
		jv *a = jnew('a');
		jp++;
		for (;;) {
			skip_ws();
			if (*jp == ']') { jp++; break; }
			jpush(a, NULL, jparse());
			skip_ws();
			if (*jp == ',') jp++;
		}
		return a;
	}
	if (*jp == '"') {
		jv *s = jnew('s');
		s->str = jstring();
		return s;
	}
	jv *n = jnew('n');
	char *end;
	n->num = strtod(jp, &end);
	if (end == jp) end++;           /* true, false, null: not used */
	jp = end;
	return n;
}

static void jfree(jv *v)
{
	if (!v) return;
	for (int i = 0; i < v->n; i++) {
		jfree(v->items[i]);
		free(v->keys[i]);
	}
	free(v->items);
	free(v->keys);
	free(v->str);
	free(v);
}

static jv *jget(jv *o, const char *key)
{
	if (!o || o->type != 'o') return NULL;
	for (int i = 0; i < o->n; i++)
		if (!strcmp(o->keys[i], key)) return o->items[i];
	return NULL;
}

static uint32_t jnum(jv *o, const char *key)
{
	jv *v = jget(o, key);
	return v ? (uint32_t)(int64_t)v->num : 0;
}

static char *read_gz(const char *path)
{
	gzFile f = gzopen(path, "rb");
	if (!f) return NULL;
	size_t cap = 1 << 22, len = 0;
	char *buf = malloc(cap);
	int got;
	while ((got = gzread(f, buf + len, (unsigned)(cap - len - 1))) > 0) {
		len += (size_t)got;
		if (cap - len < (1 << 20)) {
			cap *= 2;
			buf = realloc(buf, cap);
		}
	}
	gzclose(f);
	buf[len] = 0;
	return buf;
}

/* ------------------------------------------------------------------ */
/* Running one test                                                    */
/* ------------------------------------------------------------------ */

static uint8_t *mem;

/* Addresses touched by the last test, cleared before the next one */
static uint32_t *touched;
static size_t n_touched, cap_touched;

static void touch(uint32_t a)
{
	if (n_touched == cap_touched) {
		cap_touched = cap_touched ? cap_touched * 2 : 4096;
		touched = realloc(touched, cap_touched * sizeof(uint32_t));
	}
	touched[n_touched++] = a & 0xffffff;
}

static void poke(uint32_t a, uint8_t v)
{
	mem[a & 0xffffff] = v;
	touch(a);
}

static void load_state(jv *st)
{
	char k[4];
	for (int i = 0; i < 8; i++) {
		sprintf(k, "d%d", i);
		reg[i] = (w32)jnum(st, k);
	}
	for (int i = 0; i < 7; i++) {
		sprintf(k, "a%d", i);
		aReg[i] = (w32)jnum(st, k);
	}
	uint16_t sr = (uint16_t)jnum(st, "sr");
	usp = (w32)jnum(st, "usp");
	ssp = (w32)jnum(st, "ssp");

	/* The SR, variable by variable: PutSR would also take interrupts */
	trace = (sr & 0x8000) != 0;
	supervisor = (sr & 0x2000) != 0;
	iMask = (char)((sr >> 8) & 7);
	xflag = (sr & 0x10) != 0;
	negative = (sr & 0x08) != 0;
	zero = (sr & 0x04) != 0;
	overflow = (sr & 0x02) != 0;
	carry = (sr & 0x01) != 0;
	aReg[7] = supervisor ? ssp : usp;

	/* Memory, and the two prefetched words: the start of the instruction */
	jv *ram = jget(st, "ram");
	for (int i = 0; ram && i < ram->n; i++) {
		jv *e = ram->items[i];
		if (e->type == 'a' && e->n >= 2)
			poke((uint32_t)e->items[0]->num, (uint8_t)e->items[1]->num);
	}
	uint32_t pcv = jnum(st, "pc");
	jv *pf = jget(st, "prefetch");
	for (int i = 0; pf && i < pf->n && i < 2; i++) {
		uint32_t a = pcv + 2 * i, w = (uint32_t)pf->items[i]->num;
		poke(a, (uint8_t)(w >> 8));
		poke(a + 1, (uint8_t)w);
	}
}

/* Execute the instruction at the initial pc, as ExecuteLoop does */
static void run_one(uint32_t start)
{
	exception = 0;
	extraFlag = false;
	pendingInterrupt = 0;
	stopped = false;
	badCodeAddress = false;
	doTrace = trace;        /* the trace exception follows the instruction */
	saved_sr = GetSR();     /* as ExecuteLoop: the SR an address error restores */
	nInst = 1;
	nInst2 = 0;

	pc = (uw16 *)((Ptr)memBase + (start & 0xffffff));
	code = RW(pc++) & 0xffff;
	nInst--;
	qlux_table[code]();

	if (extraFlag || doTrace) {
		nInst = nInst2;
		ExceptionProcessing();
	}
}

/* ------------------------------------------------------------------ */
/* Comparing                                                           */
/* ------------------------------------------------------------------ */

enum { F_D0, F_A0 = 8, F_USP = 15, F_SSP, F_SR_C, F_SR_V, F_SR_Z, F_SR_N,
       F_SR_X, F_SR_OTHER, F_PC, F_RAM, F_COUNT };
static const char *fname(int f)
{
	static char s[8];
	if (f < 8) { sprintf(s, "D%d", f); return s; }
	if (f < 15) { sprintf(s, "A%d", f - 8); return s; }
	static const char *n[] = { "USP", "SSP", "C", "V", "Z", "N", "X",
				   "SR T/S/I", "PC", "memory" };
	return n[f - 15];
}

/* Returns a bit mask of the fields that differ */
static uint32_t compare(jv *fin, char *why, size_t whylen)
{
	uint32_t bad = 0;
	char k[4];
	why[0] = 0;
#define NOTE(...) do { size_t l = strlen(why); if (l < whylen - 80) \
	snprintf(why + l, whylen - l, __VA_ARGS__); } while (0)

	for (int i = 0; i < 8; i++) {
		sprintf(k, "d%d", i);
		uint32_t e = jnum(fin, k), g = (uint32_t)reg[i];
		if (e != g) { bad |= 1u << (F_D0 + i); NOTE(" D%d %08X/%08X", i, g, e); }
	}
	for (int i = 0; i < 7; i++) {
		sprintf(k, "a%d", i);
		uint32_t e = jnum(fin, k), g = (uint32_t)aReg[i];
		if (e != g) { bad |= 1u << (F_A0 + i); NOTE(" A%d %08X/%08X", i, g, e); }
	}
	uint32_t gusp = supervisor ? (uint32_t)usp : (uint32_t)aReg[7];
	uint32_t gssp = supervisor ? (uint32_t)aReg[7] : (uint32_t)ssp;
	if (gusp != jnum(fin, "usp")) { bad |= 1u << F_USP; NOTE(" USP %08X/%08X", gusp, jnum(fin, "usp")); }
	if (gssp != jnum(fin, "ssp")) { bad |= 1u << F_SSP; NOTE(" SSP %08X/%08X", gssp, jnum(fin, "ssp")); }

	uint16_t gsr = (uint16_t)GetSR(), esr = (uint16_t)jnum(fin, "sr");
	static const int bits[5] = { 0x01, 0x02, 0x04, 0x08, 0x10 };
	for (int i = 0; i < 5; i++)
		if ((gsr ^ esr) & bits[i]) bad |= 1u << (F_SR_C + i);
	if ((gsr ^ esr) & 0xa700) bad |= 1u << F_SR_OTHER;
	if ((gsr ^ esr) & 0xa71f) NOTE(" SR %04X/%04X", gsr, esr);

	uint32_t gpc = (uint32_t)((Ptr)pc - (Ptr)memBase);
	if ((gpc & 0xffffff) != (jnum(fin, "pc") & 0xffffff)) {
		bad |= 1u << F_PC;
		NOTE(" PC %06X/%06X", gpc & 0xffffff, jnum(fin, "pc") & 0xffffff);
	}

	jv *ram = jget(fin, "ram");
	int shown = 0;
	for (int i = 0; ram && i < ram->n; i++) {
		jv *e = ram->items[i];
		if (e->type != 'a' || e->n < 2) continue;
		uint32_t a = (uint32_t)e->items[0]->num & 0xffffff;
		uint8_t v = (uint8_t)e->items[1]->num;
		touch(a);
		if (mem[a] != v) {
			bad |= 1u << F_RAM;
			if (shown++ < 3) NOTE(" [%06X] %02X/%02X", a, mem[a], v);
		}
	}
	return bad;
}

/* ------------------------------------------------------------------ */

static int show = 3;            /* failing tests shown per file */
static int show_ae = 0;         /* -a: show the failing address errors */

/* Does the test expect an address error? Its final PC is then the address
 * error handler, the long word at $0C in the initial memory. */
static int expects_address_error(jv *ini, jv *fin)
{
	jv *ram = jget(ini, "ram");
	uint8_t v[4];
	int found = 0;
	for (int i = 0; ram && i < ram->n; i++) {
		jv *e = ram->items[i];
		if (e->type != 'a' || e->n < 2) continue;
		uint32_t a = (uint32_t)e->items[0]->num & 0xffffff;
		if (a >= 0x0c && a <= 0x0f) {
			v[a - 0x0c] = (uint8_t)e->items[1]->num;
			found |= 1 << (a - 0x0c);
		}
	}
	if (found != 15) return 0;
	uint32_t handler = ((uint32_t)v[0] << 24 | v[1] << 16 | v[2] << 8 | v[3]) & 0xffffff;
	return (jnum(fin, "pc") & 0xffffff) == handler;
}

static unsigned long tot_ae_ok, tot_ae_bad;

static void run_file(const char *path, unsigned long *tot_ok, unsigned long *tot_bad)
{
	char *text = read_gz(path);
	if (!text) { fprintf(stderr, "cannot read %s\n", path); return; }
	jp = text;
	jv *tests = jparse();
	free(text);
	if (!tests || tests->type != 'a') { fprintf(stderr, "%s: not a test array\n", path); jfree(tests); return; }

	unsigned long ok = 0, bad = 0, field[F_COUNT] = { 0 };
	unsigned long ae_ok = 0, ae_bad = 0;
	int shown = 0;
	for (int t = 0; t < tests->n; t++) {
		jv *test = tests->items[t];
		jv *ini = jget(test, "initial"), *fin = jget(test, "final");
		if (!ini || !fin) continue;

		for (size_t i = 0; i < n_touched; i++) mem[touched[i]] = 0;
		n_touched = 0;

		load_state(ini);
		run_one(jnum(ini, "pc"));

		char why[512];
		uint32_t m = compare(fin, why, sizeof(why));
		/* address errors are counted apart, so that they do not hide
		 * the other differences */
		if (expects_address_error(ini, fin)) {
			if (!m) { ae_ok++; continue; }
			ae_bad++;
			if (show_ae && shown++ < show) {
				jv *name = jget(test, "name");
				printf("    %-34s got/expected:%s\n", name && name->str ? name->str : "?", why);
			}
			continue;
		}
		if (!m) { ok++; continue; }
		bad++;
		for (int f = 0; f < F_COUNT; f++)
			if (m & (1u << f)) field[f]++;
		if (shown++ < show) {
			jv *name = jget(test, "name");
			printf("    %-34s got/expected:%s\n", name && name->str ? name->str : "?", why);
		}
	}
	jfree(tests);

	const char *base = strrchr(path, '/');
	base = base ? base + 1 : path;
	printf("%-24s %6lu ok %6lu bad", base, ok, bad);
	if (ae_ok + ae_bad)
		printf("   address errors: %lu ok %lu bad", ae_ok, ae_bad);
	if (bad) {
		printf("   fields:");
		for (int f = 0; f < F_COUNT; f++)
			if (field[f]) printf(" %s:%lu", fname(f), field[f]);
	}
	printf("\n");
	*tot_ok += ok;
	*tot_bad += bad;
	tot_ae_ok += ae_ok;
	tot_ae_bad += ae_bad;
}

int main(int argc, char **argv)
{
	int first = 1;
	while (first < argc && argv[first][0] == '-') {
		if (!strcmp(argv[first], "-a")) {
			show_ae = 1;
			first++;
		} else if (!strcmp(argv[first], "-v") && first + 1 < argc) {
			show = atoi(argv[first + 1]);
			first += 2;
		} else
			break;
	}
	if (first >= argc) {
		fprintf(stderr, "usage: %s [-a] [-v shown] tests.json.gz...\n", argv[0]);
		return 1;
	}

	memBase = calloc(1, MEM_SIZE + 64);
	mem = (uint8_t *)memBase;
	RTOP = MEM_SIZE;
	if (!memBase || EmulatorTable()) {
		fprintf(stderr, "cannot set up the CPU\n");
		return 1;
	}
	/* sQLux installs its own opcodes for QDOS calls: back to a plain 68000 */
	qlux_table[0x4e40] = trap0;
	qlux_table[0x4e41] = trap1;
	qlux_table[0x4e42] = trap2;
	qlux_table[0x4e43] = trap3;

	unsigned long ok = 0, bad = 0;
	for (int i = first; i < argc; i++)
		run_file(argv[i], &ok, &bad);
	printf("\nTotal without address errors: %lu ok, %lu bad (%.2f %% ok)\n",
	       ok, bad, ok + bad ? 100.0 * ok / (ok + bad) : 0.0);
	printf("Address errors: %lu ok, %lu bad\n", tot_ae_ok, tot_ae_bad);
	return bad != 0;
}
