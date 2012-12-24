/****************************************************************************
Copyright (c) 2006 - 2012, Armin Biere, Johannes Kepler University.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <ctype.h>
#include <stdarg.h>

#include "picosat.h"

/* By default code for 'all different constraints' is disabled, since 'NADC'
 * is defined.
 */
#define NADC

/* By default we enable failed literals, since 'NFL' is undefined.
 *
#define NFL
 */

/* By default we 'detach satisfied (large) clauses', e.g. NDSC undefined.
 *
#define NDSC
 */

/* Do not use luby restart schedule instead of inner/outer.
 *
#define NLUBY
 */

/* Enabling this define, will use gnuplot to visualize how the scores evolve.
 *
#define VISCORES 
 */
#ifdef VISCORES
// #define WRITEGIF		/* ... to generate a video */
#endif

#ifdef VISCORES
#ifndef WRITEGIF
#include <unistd.h>		/* for 'usleep' */
#endif
#endif

#define MINRESTART	100	/* minimum restart interval */
#define MAXRESTART	1000000 /* maximum restart interval */
#define RDECIDE		1000	/* interval of random decisions */
#define FRESTART	110	/* restart increase factor in percent */
#define FREDUCE		110	/* reduce increase factor in percent  */
#define FREDADJ		121	/* reduce increase adjustment factor */
#define MAXCILS		10	/* maximal number of unrecycled internals */
#define FFLIPPED	10000	/* flipped reduce factor */
#define FFLIPPEDPREC	10000000/* flipped reduce factor precision */

#ifndef TRACE
#define NO_BINARY_CLAUSES	/* store binary clauses more compactly */
#endif

/* For debugging purposes you may want to define 'LOGGING', which actually
 * can be enforced by using the '--log' option for the configure script.
 */
#ifdef LOGGING
#define LOG(code) do { code; } while (0)
#else
#define LOG(code) do { } while (0)
#endif
#define NOLOG(code) do { } while (0)		/* log exception */
#define ONLYLOG(code) do { code; } while (0)	/* force logging */

#define FALSE ((Val)-1)
#define UNDEF ((Val)0)
#define TRUE ((Val)1)

#define COMPACT_TRACECHECK_TRACE_FMT 0
#define EXTENDED_TRACECHECK_TRACE_FMT 1
#define RUP_TRACE_FMT 2

#define NEWN(p,n) do { (p) = new (sizeof (*(p)) * (n)); } while (0)
#define CLRN(p,n) do { memset ((p), 0, sizeof (*(p)) * (n)); } while (0)
#define CLR(p) CLRN(p,1)
#define DELETEN(p,n) \
  do { delete (p, sizeof (*(p)) * (n)); (p) = 0; } while (0)

#define RESIZEN(p,old_num,new_num) \
  do { \
    size_t old_size = sizeof (*(p)) * (old_num); \
    size_t new_size = sizeof (*(p)) * (new_num); \
    (p) = resize ((p), old_size, new_size) ; \
  } while (0)

#define ENLARGE(start,head,end) \
  do { \
    unsigned old_num = (unsigned)((end) - (start)); \
    size_t new_num = old_num ? (2 * old_num) : 1; \
    unsigned count = (head) - (start); \
    assert ((start) <= (end)); \
    RESIZEN((start),old_num,new_num); \
    (head) = (start) + count; \
    (end) = (start) + new_num; \
  } while (0)

#define NOTLIT(l) (lits + (1 ^ ((l) - lits)))

#define LIT2IDX(l) ((unsigned)((l) - lits) / 2)
#define LIT2IMPLS(l) (impls + (unsigned)((l) - lits))
#define LIT2INT(l) ((int)(LIT2SGN(l) * LIT2IDX(l)))
#define LIT2SGN(l) (((unsigned)((l) - lits) & 1) ? -1 : 1)
#define LIT2VAR(l) (vars + LIT2IDX(l))
#define LIT2HTPS(l) (htps + (unsigned)((l) - lits))
#define LIT2JWH(l) (jwh + ((l) - lits))

#ifndef NDSC
#define LIT2DHTPS(l) (dhtps + (unsigned)((l) - lits))
#endif

#ifdef NO_BINARY_CLAUSES
typedef unsigned long Wrd;
#define ISLITREASON(cls) (1&(Wrd)cls)
#define LIT2REASON(lit) \
  (assert (lit->val==TRUE), ((Cls*)(1 + (2*(lit - lits)))))
#define REASON2LIT(cls) ((Lit*)(lits + ((Wrd)cls)/2))
#endif

#define ENDOFCLS(c) ((void*)((c)->lits + (c)->size))

#define SOC ((oclauses == ohead) ? lclauses : oclauses)
#define EOC lhead
#define NXC(p) (((p) + 1 == ohead) ? lclauses : (p) + 1)

#define OIDX2IDX(idx) (2 * ((idx) + 1))
#define LIDX2IDX(idx) (2 * (idx) + 1)

#define ISLIDX(idx) ((idx)&1)

#define IDX2OIDX(idx) (assert(!ISLIDX(idx)), (idx)/2 - 1)
#define IDX2LIDX(idx) (assert(ISLIDX(idx)), (idx)/2)

#define EXPORTIDX(idx) \
  ((ISLIDX(idx) ? (IDX2LIDX (idx) + (ohead - oclauses)) : IDX2OIDX(idx)) + 1)

#define IDX2CLS(i) \
  (assert(i), (ISLIDX(i) ? lclauses : oclauses)[(i)/2 - !ISLIDX(i)])

#define IDX2ZHN(i) (assert(i), (ISLIDX(i) ? zhains[(i)/2] : 0))

#define CLS2TRD(c) (((Trd*)(c)) - 1)
#define CLS2IDX(c) ((((Trd*)(c)) - 1)->idx)

#define CLS2ACT(c) \
  ((Act*)((assert((c)->learned)),assert((c)->size>2),ENDOFCLS(c)))

#define VAR2LIT(v) (lits + 2 * ((v) - vars))
#define VAR2RNK(v) (rnks + ((v) - vars))

#define RNK2LIT(r) (lits + 2 * ((r) - rnks))
#define RNK2VAR(r) (vars + ((r) - rnks))

#define BLK_FILL_BYTES 8
#define SIZE_OF_BLK (sizeof (Blk) - BLK_FILL_BYTES)

#define PTR2BLK(void_ptr) \
  ((void_ptr) ? (Blk*)(((char*)(void_ptr)) - SIZE_OF_BLK) : 0)

#define AVERAGE(a,b) ((b) ? (((double)a) / (double)(b)) : 0.0)
#define PERCENT(a,b) (100.0 * AVERAGE(a,b))

#define ABORT(msg) \
  do { \
    fputs ("*** picosat: " msg "\n", stderr); \
    abort (); \
  } while (0)

#define ABORTIF(cond,msg) \
  do { \
    if (!(cond)) break; \
    ABORT (msg); \
  } while (0)

#define ZEROFLT		(0x00000000u)
#define EPSFLT		(0x00000001u)
#define INFFLT		(0xffffffffu)

#define FLTCARRY	(1u << 25)
#define FLTMSB		(1u << 24)
#define FLTMAXMANTISSA	(FLTMSB - 1)

#define FLTMANTISSA(d)	((d) & FLTMAXMANTISSA)
#define FLTEXPONENT(d)	((int)((d) >> 24) - 128)

#define FLTMINEXPONENT	(-128)
#define FLTMAXEXPONENT	(127)

#define CMPSWAPFLT(a,b) \
  do { \
    Flt tmp; \
    if (((a) < (b))) \
      { \
	tmp = (a); \
	(a) = (b); \
	(b) = tmp; \
      } \
  } while (0)

#define UNPACKFLT(u,m,e) \
  do { \
    (m) = FLTMANTISSA(u); \
    (e) = FLTEXPONENT(u); \
    (m) |= FLTMSB; \
  } while (0)

#define INSERTION_SORT_LIMIT 10

#define SORTING_SWAP(T,p,q) \
do { \
  T tmp = *(q); \
  *(q) = *(p); \
  *(p) = tmp; \
} while (0)

#define SORTING_CMP_SWAP(T,cmp,p,q) \
do { \
  if ((cmp) (*(p), *(q)) > 0) \
    SORTING_SWAP (T, p, q); \
} while(0)

#define QUICKSORT_PARTITION(T,cmp,a,l,r) \
do { \
  T pivot; \
  int j; \
  i = (l) - 1; 			/* result in 'i' */ \
  j = (r); \
  pivot = (a)[j]; \
  for (;;) \
    { \
      while ((cmp) ((a)[++i], pivot) < 0) \
	; \
      while ((cmp) (pivot, (a)[--j]) < 0) \
        if (j == (l)) \
	  break; \
      if (i >= j) \
	break; \
      SORTING_SWAP (T, (a) + i, (a) + j); \
    } \
  SORTING_SWAP (T, (a) + i, (a) + (r)); \
} while(0)

#define QUICKSORT(T,cmp,a,n) \
do { \
  int l = 0, r = (n) - 1, m, ll, rr, i; \
  assert (ihead == indices); \
  if (r - l <= INSERTION_SORT_LIMIT) \
    break; \
  for (;;) \
    { \
      m = (l + r) / 2; \
      SORTING_SWAP (T, (a) + m, (a) + r - 1); \
      SORTING_CMP_SWAP (T, cmp, (a) + l, (a) + r - 1); \
      SORTING_CMP_SWAP (T, cmp, (a) + l, (a) + r); \
      SORTING_CMP_SWAP (T, cmp, (a) + r - 1, (a) + r); \
      QUICKSORT_PARTITION (T, cmp, (a), l + 1, r - 1); \
      if (i - l < r - i) \
	{ \
	  ll = i + 1; \
	  rr = r; \
	  r = i - 1; \
	} \
      else \
	{ \
	  ll = l; \
	  rr = i - 1; \
	  l = i + 1; \
	} \
      if (r - l > INSERTION_SORT_LIMIT) \
	{ \
	  assert (rr - ll > INSERTION_SORT_LIMIT); \
	  if (ihead == eoi) \
	    ENLARGE (indices, ihead, eoi); \
	  *ihead++ = ll; \
	  if (ihead == eoi) \
	    ENLARGE (indices, ihead, eoi); \
	  *ihead++ = rr; \
	} \
      else if (rr - ll > INSERTION_SORT_LIMIT) \
        { \
	  l = ll; \
	  r = rr; \
	} \
      else if (ihead > indices) \
	{ \
	  r = *--ihead; \
	  l = *--ihead; \
	} \
      else \
	break; \
    } \
} while (0)

#define INSERTION_SORT(T,cmp,a,n) \
do { \
  T pivot; \
  int l = 0, r = (n) - 1, i, j; \
  for (i = r; i > l; i--) \
    SORTING_CMP_SWAP (T, cmp, (a) + i - 1, (a) + i); \
  for (i = l + 2; i <= r; i++)  \
    { \
      j = i; \
      pivot = (a)[i]; \
      while ((cmp) (pivot, (a)[j - 1]) < 0) \
        { \
	  (a)[j] = (a)[j - 1]; \
	  j--; \
	} \
      (a)[j] = pivot; \
    } \
} while (0)

#ifdef NDEBUG
#define CHECK_SORTED(cmp,a,n) do { } while(0)
#else
#define CHECK_SORTED(cmp,a,n) \
do { \
  int i; \
  for (i = 0; i < (n) - 1; i++) \
    assert ((cmp) ((a)[i], (a)[i + 1]) <= 0); \
} while(0)
#endif

#define SORT(T,cmp,a,n) \
do { \
  T * aa = (a); \
  int nn = (n); \
  QUICKSORT (T, cmp, aa, nn); \
  INSERTION_SORT (T, cmp, aa, nn); \
  assert (ihead == indices); \
  CHECK_SORTED (cmp, aa, nn); \
} while (0)

#define WRDSZ (sizeof (long) * 8)

typedef unsigned Flt;		/* 32 bit deterministic soft float */
typedef Flt Act;		/* clause and variable activity */
typedef struct Blk Blk;		/* allocated memory block */
typedef struct Cls Cls;		/* clause */
typedef struct Lit Lit;		/* literal */
typedef struct Rnk Rnk;		/* variable to score mapping */
typedef signed char Val;	/* TRUE, UNDEF, FALSE */
typedef struct Var Var;		/* variable */
#ifdef TRACE
typedef struct Trd Trd;		/* trace data for clauses */
typedef struct Zhn Zhn;		/* compressed chain (=zain) data */
typedef unsigned char Znt;	/* compressed antecedent data */
#endif

#ifdef NO_BINARY_CLAUSES
typedef struct Ltk Ltk;

struct Ltk
{
  Lit ** start;
  unsigned count : WRDSZ == 32 ? 27 : 32;
  unsigned ldsize : WRDSZ == 32 ? 5 : 32;
};
#endif

struct Lit
{
  Val val;
};

struct Var
{
  unsigned mark		: 1;	/*bit 1*/
  unsigned resolved	: 1;	/*bit 2*/
  unsigned phase	: 1;	/*bit 3*/
  unsigned assigned	: 1;	/*bit 4*/
  unsigned used		: 1;	/*bit 5*/
  unsigned failed	: 1;	/*bit 6*/
  unsigned internal	: 1;	/*bit 7*/
  unsigned usedefphase  : 1;    /*bit 8*/
  unsigned defphase     : 1;    /*bit 9*/
  unsigned msspos       : 1;    /*bit 10*/
  unsigned mssneg       : 1;    /*bit 11*/
  unsigned humuspos     : 1;    /*bit 12*/
  unsigned humusneg     : 1;    /*bit 13*/
  unsigned partial      : 1;    /*bit 14*/
#ifdef TRACE
  unsigned core		: 1;	/*bit 15*/
#endif
  unsigned level;
  Cls *reason;
#ifndef NADC
  Lit ** inado;
  Lit ** ado;
  Lit *** adotabpos;
#endif
};

struct Rnk
{
  Act score;
  unsigned pos : 30;			/* 0 iff not on heap */
  unsigned moreimportant : 1;
  unsigned lessimportant : 1;
};

struct Cls
{
  unsigned size;

  unsigned collect:1;
  unsigned learned:1;
  unsigned locked:1;
  unsigned used:1;
#ifndef NDEBUG
  unsigned connected:1;
#endif
#ifdef TRACE
  unsigned collected:1;
  unsigned core:1;
#endif

  Cls *next[2];
  Lit *lits[2];
};

#ifdef TRACE
struct Zhn
{
  unsigned ref:31;
  unsigned core:1;
  Znt * liz;
  Znt znt[0];
};

struct Trd
{
  unsigned idx;
  Cls cls[0];
};
#endif

struct Blk
{
#ifndef NDEBUG
  union
  {
    size_t size;		/* this is what we really use */
    void *as_two_ptrs[2];	/* 2 * sizeof (void*) alignment of data */
  }
  header;
#endif
  char data[BLK_FILL_BYTES];
};

static enum State
{
  RESET = 0,
  READY = 1,
  SAT = 2,
  UNSAT = 3,
  UNKNOWN = 4,
} 
state = RESET;

static enum Phase
{
  POSPHASE,
  NEGPHASE,
  JWLPHASE,
  RNDPHASE,
}
defaultphase = JWLPHASE;

static int last_sat_call_result;

static FILE *out;
static char * prefix;
static int verbosity;
static unsigned level;
static unsigned max_var;
static unsigned size_vars;

static Lit * lits;
static Var *vars;
static Rnk *rnks;
static Flt *jwh;
static Cls **htps;
#ifndef NDSC
static Cls **dhtps;
#endif
#ifdef NO_BINARY_CLAUSES
static Ltk *impls;
static Cls impl, cimpl;
static int implvalid, cimplvalid;
#else
static Cls **impls;
#endif
static Lit **trail, **thead, **eot, **ttail, ** ttail2;
#ifndef NADC
static Lit **ttailado;
#endif
static unsigned adecidelevel;
static Lit **als, **alshead, **alstail, **eoals;
static Lit **cls, **clshead, **eocls;
static int *rils, *rilshead, *eorils;
static int *cils, *cilshead, *eocils;
static int *fals, *falshead, *eofals;
static int *mass, szmass;
static int *mssass, szmssass;
static int *mcsass, nmcsass, szmcsass;
static int *humus, szhumus;
static Lit *failed_assumption;
static int extracted_all_failed_assumptions;
static Rnk **heap, **hhead, **eoh;
static Cls **oclauses, **ohead, **eoo;	/* original clauses */
static Cls **lclauses, **lhead, ** eol;	/* learned clauses */
static int * soclauses, * sohead, * eoso; /* saved original clauses */
static int saveorig;
static int partial;
#ifdef TRACE
static int trace;
static Zhn **zhains, **zhead, **eoz;
static int ocore = -1;
#endif
static FILE * rup;
static int rupstarted;
static int rupvariables;
static int rupclauses;
static Cls *mtcls;
static Cls *conflict;
static Lit **added, **ahead, **eoa;
static Var **marked, **mhead, **eom;
static Var **dfs, **dhead, **eod;
static Cls **resolved, **rhead, **eor;
static unsigned char *buffer, *bhead, *eob;
static Act vinc, lscore, ilvinc, ifvinc;
#ifdef VISCORES
static Act fvinc, nvinc;
#endif
static Act cinc, lcinc, ilcinc, fcinc;
static unsigned srng;
static size_t current_bytes;
static size_t max_bytes;
static size_t recycled;
static double seconds, flseconds;
static double entered;
static unsigned nentered;
static int measurealltimeinlib;
static char *rline[2];
static int szrline, rcount;
static double levelsum;
static unsigned iterations;
static int reports;
static int lastrheader = -2;
static unsigned calls;
static unsigned decisions;
static unsigned restarts;
static unsigned simps;
static unsigned fsimplify;
static unsigned isimplify;
static unsigned reductions;
static unsigned lreduce;
static unsigned lreduceadjustcnt;
static unsigned lreduceadjustinc;
static unsigned lastreduceconflicts;
static unsigned llocked;	/* locked large learned clauses */
static unsigned lrestart;
#ifdef NLUBY
static unsigned drestart;
static unsigned ddrestart;
#else
static unsigned lubycnt;
static unsigned lubymaxdelta;
static int waslubymaxdelta;
#endif
static unsigned long long lsimplify;
static unsigned long long propagations;
static unsigned long long lpropagations;
static unsigned fixed;		/* top level assignments */
#ifndef NFL
static unsigned failedlits;
static unsigned ifailedlits;
static unsigned efailedlits;
static unsigned flcalls;
#ifdef STATS
static unsigned flrounds;
static unsigned long long flprops;
static unsigned long long floopsed, fltried, flskipped;
#endif
static unsigned long long fllimit;
static int simplifying;
static Lit ** saved;
static unsigned saved_size;
#endif
static unsigned conflicts;
static unsigned contexts;
static unsigned internals;
static unsigned noclauses;	/* current number large original clauses */
static unsigned nlclauses;	/* current number large learned clauses */
static unsigned olits;		/* current literals in large original clauses */
static unsigned llits;		/* current literals in large learned clauses */
static unsigned oadded;		/* added original clauses */
static unsigned ladded;		/* added learned clauses */
static unsigned loadded;	/* added original large clauses */
static unsigned lladded;	/* added learned large clauses */
static unsigned addedclauses;	/* oadded + ladded */
static unsigned vused;		/* used variables */
static unsigned llitsadded;	/* added learned literals */
static unsigned long long visits;
#ifdef STATS
static unsigned loused;		/* used large original clauses */
static unsigned llused;		/* used large learned clauses */
static unsigned long long bvisits;
static unsigned long long tvisits;
static unsigned long long lvisits;
static unsigned long long othertrue;
static unsigned long long othertrue2;
static unsigned long long othertruel;
static unsigned long long othertrue2u;
static unsigned long long othertruelu;
static unsigned long long ltraversals;
static unsigned long long traversals;
#ifdef TRACE
static unsigned long long antecedents;
#endif
static unsigned uips;
static unsigned znts;
static unsigned assumptions;
static unsigned rdecisions;
static unsigned sdecisions;
static size_t srecycled;
static size_t rrecycled;
static unsigned long long derefs;
#endif
static unsigned minimizedllits;
static unsigned nonminimizedllits;
#ifndef NADC
static Lit *** ados, *** hados, *** eados;
static Lit *** adotab;
static unsigned nadotab;
static unsigned szadotab;
static Cls * adoconflict;
static unsigned adoconflicts;
static unsigned adoconflictlimit = UINT_MAX;
static int addingtoado;
static int adodisabled;
#endif
static unsigned long long flips;
#ifdef STATS
static unsigned long long forced;
static unsigned long long assignments;
static unsigned inclreduces;
static unsigned staticphasedecisions;
static unsigned skippedrestarts;
#endif
static int * indices, * ihead, *eoi; 
static unsigned sdflips;

static unsigned long long saved_flips;
static unsigned saved_max_var;
static unsigned min_flipped = UINT_MAX;

static void * emgr;
static void * (*enew)(void*,size_t);
static void * (*eresize)(void*,void*,size_t,size_t);
static void (*edelete)(void*,void*,size_t);

#ifdef VISCORES
static FILE * fviscores;
#endif

static Flt
packflt (unsigned m, int e)
{
  Flt res;
  assert (m < FLTMSB);
  assert (FLTMINEXPONENT <= e);
  assert (e <= FLTMAXEXPONENT);
  res = m | ((e + 128) << 24);
  return res;
}

static Flt
base2flt (unsigned m, int e)
{
  if (!m)
    return ZEROFLT;

  if (m < FLTMSB)
    {
      do
	{
	  if (e <= FLTMINEXPONENT)
	    return EPSFLT;

	  e--;
	  m <<= 1;

	}
      while (m < FLTMSB);
    }
  else
    {
      while (m >= FLTCARRY)
	{
	  if (e >= FLTMAXEXPONENT)
	    return INFFLT;

	  e++;
	  m >>= 1;
	}
    }

  m &= ~FLTMSB;
  return packflt (m, e);
}

static Flt
addflt (Flt a, Flt b)
{
  unsigned ma, mb, delta;
  int ea, eb;

  CMPSWAPFLT (a, b);
  if (!b)
    return a;

  UNPACKFLT (a, ma, ea);
  UNPACKFLT (b, mb, eb);

  assert (ea >= eb);
  delta = ea - eb;
  mb >>= delta;
  if (!mb)
    return a;

  ma += mb;
  if (ma & FLTCARRY)
    {
      if (ea == FLTMAXEXPONENT)
	return INFFLT;

      ea++;
      ma >>= 1;
    }

  assert (ma < FLTCARRY);
  ma &= FLTMAXMANTISSA;

  return packflt (ma, ea);
}

static Flt
mulflt (Flt a, Flt b)
{
  unsigned ma, mb;
  unsigned long long accu;
  int ea, eb;

  CMPSWAPFLT (a, b);
  if (!b)
    return ZEROFLT;

  UNPACKFLT (a, ma, ea);
  UNPACKFLT (b, mb, eb);

  ea += eb;
  ea += 24;
  if (ea > FLTMAXEXPONENT)
    return INFFLT;

  if (ea < FLTMINEXPONENT)
    return EPSFLT;

  accu = ma;
  accu *= mb;
  accu >>= 24;

  if (accu >= FLTCARRY)
    {
      if (ea == FLTMAXEXPONENT)
	return INFFLT;

      ea++;
      accu >>= 1;

      if (accu >= FLTCARRY)
	return INFFLT;
    }

  assert (accu < FLTCARRY);
  assert (accu & FLTMSB);

  ma = accu;
  ma &= ~FLTMSB;

  return packflt (ma, ea);
}

static Flt
ascii2flt (const char *str)
{
  Flt ten = base2flt (10, 0);
  Flt onetenth = base2flt (26843546, -28);
  Flt res = ZEROFLT, tmp, base;
  const char *p = str;
  char ch;

  ch = *p++;

  if (ch != '.')
    {
      if (!isdigit (ch))
	return INFFLT;	/* better abort ? */

      res = base2flt (ch - '0', 0);

      while ((ch = *p++))
	{
	  if (ch == '.')
	    break;

	  if (!isdigit (ch))
	    return INFFLT;	/* better abort? */

	  res = mulflt (res, ten);
	  tmp = base2flt (ch - '0', 0);
	  res = addflt (res, tmp);
	}
    }

  if (ch == '.')
    {
      ch = *p++;
      if (!isdigit (ch))
	return INFFLT;	/* better abort ? */

      base = onetenth;
      tmp = mulflt (base2flt (ch - '0', 0), base);
      res = addflt (res, tmp);

      while ((ch = *p++))
	{
	  if (!isdigit (ch))
	    return INFFLT;	/* better abort? */

	  base = mulflt (base, onetenth);
	  tmp = mulflt (base2flt (ch - '0', 0), base);
	  res = addflt (res, tmp);
	}
    }

  return res;
}

#if defined(VISCORES)

static double
flt2double (Flt f)
{
  double res;
  unsigned m;
  int e, i;

  UNPACKFLT (f, m, e);
  res = m;

  if (e < 0)
    {
      for (i = e; i < 0; i++)
	res *= 0.5;
    }
  else
    {
      for (i = 0; i < e; i++)
	res *= 2.0;
    }

  return res;
}

#endif

static int
log2flt (Flt a)
{
  return FLTEXPONENT (a) + 24;
}

static int
cmpflt (Flt a, Flt b)
{
  if (a < b)
    return -1;

  if (a > b)
    return 1;

  return 0;
}

static void *
new (size_t size)
{
  size_t bytes;
  Blk *b;
  
  if (!size)
    return 0;

  bytes = size + SIZE_OF_BLK;

  if (enew)
    b = enew (emgr, bytes);
  else
    b = malloc (bytes);

  ABORTIF (!b, "out of memory in 'new'");
#ifndef NDEBUG
  b->header.size = size;
#endif
  current_bytes += size;
  if (current_bytes > max_bytes)
    max_bytes = current_bytes;
  return b->data;
}

static void
delete (void *void_ptr, size_t size)
{
  size_t bytes;
  Blk *b;

  if (!void_ptr)
    {
      assert (!size);
      return;
    }

  assert (size);
  b = PTR2BLK (void_ptr);

  assert (size <= current_bytes);
  current_bytes -= size;

  assert (b->header.size == size);

  bytes = size + SIZE_OF_BLK;
  if (edelete)
    edelete (emgr, b, bytes);
  else
    free (b);
}

static void *
resize (void *void_ptr, size_t old_size, size_t new_size)
{
  size_t old_bytes, new_bytes;
  Blk *b;

  b = PTR2BLK (void_ptr);

  assert (old_size <= current_bytes);
  current_bytes -= old_size;

  if ((old_bytes = old_size))
    {
      assert (old_size && b && b->header.size == old_size);
      old_bytes += SIZE_OF_BLK;
    }
  else
    assert (!b);

  if ((new_bytes = new_size))
    new_bytes += SIZE_OF_BLK;

  if (eresize)
    b = eresize (emgr, b, old_bytes, new_bytes);
  else
    b = realloc (b, new_bytes);

  if (!new_size)
    {
      assert (!b);
      return 0;
    }

  ABORTIF (!b, "out of memory in 'resize'");
#ifndef NDEBUG
  b->header.size = new_size;
#endif

  current_bytes += new_size;
  if (current_bytes > max_bytes)
    max_bytes = current_bytes;

  return b->data;
}

static unsigned
int2unsigned (int l)
{
  return (l < 0) ? 1 + 2 * -l : 2 * l;
}

static Lit *
int2lit (int l)
{
  return lits + int2unsigned (l);
}

static Lit **
end_of_lits (Cls * cls)
{
  return cls->lits + cls->size;
}

#if !defined(NDEBUG) || defined(LOGGING)

static void
dumplits (Lit ** l, Lit ** eol)
{
  int first;
  Lit ** p;

  if (l == eol)
    {
      /* empty clause */
    }
  else if (l + 1 == eol)
    {
      fprintf (out, "%d ", LIT2INT (l[0]));
    }
  else
    { 
      assert (l + 2 <= eol);
      first = (abs (LIT2INT (l[0])) > abs (LIT2INT (l[1])));
      fprintf (out, "%d ", LIT2INT (l[first]));
      fprintf (out, "%d ", LIT2INT (l[!first]));
      for (p = l + 2; p < eol; p++)
	fprintf (out, "%d ", LIT2INT (*p));
    }

  fputc ('0', out);
}

static void
dumpcls (Cls * cls)
{
  Lit **eol;

  if (cls)
    {
      eol = end_of_lits (cls);
      dumplits (cls->lits, eol);
#ifdef TRACE
      if (trace)
	fprintf (out, " clause(%u)", CLS2IDX (cls));
#endif
    }
  else
    fputs ("DECISION", out);
}

static void
dumpclsnl (Cls * cls)
{
  dumpcls (cls);
  fputc ('\n', out);
}

void
dumpcnf (void)
{
  Cls **p, *cls;

  for (p = SOC; p != EOC; p = NXC (p))
    {
      cls = *p;

      if (!cls)
	continue;

#ifdef TRACE
      if (cls->collected)
	continue;
#endif

      dumpclsnl (*p);
    }
}

#endif

static void
delete_prefix (void)
{
  if (!prefix)
    return;
    
  delete (prefix, strlen (prefix) + 1);
  prefix = 0;
}

static void
new_prefix (const char * str)
{
  delete_prefix ();
  assert (str);
  prefix = new (strlen (str) + 1);
  strcpy (prefix, str);
}

static void
init (void)
{
  static int count;

  ABORTIF (state != RESET, "API usage: multiple initializations");

  count = 3 - !enew - !eresize - !edelete;
  ABORTIF (count && !enew, "API usage: missing 'picosat_set_new'");
  ABORTIF (count && !eresize, "API usage: missing 'picosat_set_resize'");
  ABORTIF (count && !edelete, "API usage: missing 'picosat_set_delete'");

  assert (!max_var);		/* check for proper reset */
  assert (!size_vars);		/* check for proper reset */

  size_vars = 1;

  NEWN (lits, 2 * size_vars);
  NEWN (jwh, 2 * size_vars);
  NEWN (htps, 2 * size_vars);
#ifndef NDSC
  NEWN (dhtps, 2 * size_vars);
#endif
  NEWN (impls, 2 * size_vars);
  NEWN (vars, size_vars);
  NEWN (rnks, size_vars);

  ENLARGE (heap, hhead, eoh);	/* because '0' pos denotes not on heap */
  hhead = heap + 1;

  vinc = base2flt (1, 0);	/* initial variable activity */
  ifvinc = ascii2flt ("1.05");	/* variable score rescore factor */
#ifdef VISCORES
  fvinc = ascii2flt ("0.9523809");	/*     1/f =     1/1.05 */
  nvinc = ascii2flt ("0.0476191");	/* 1 - 1/f = 1 - 1/1.05 */
#endif
  lscore = base2flt (1, 90);	/* variable activity rescore limit */
  ilvinc = base2flt (1, -90);	/* inverse of 'lscore' */

  cinc = base2flt (1, 0);	/* initial clause activity */
  fcinc = ascii2flt ("1.001");	/* clause activity rescore factor */
  lcinc = base2flt (1, 90);	/* clause activity rescore limit */
  ilcinc = base2flt (1, -90);	/* inverse of 'ilcinc' */

  lreduceadjustcnt = lreduceadjustinc = 100;
  lpropagations = ~0ull;

  out = stdout;
  new_prefix ("c ");
  verbosity = 0;

#ifdef NO_BINARY_CLAUSES
  memset (&impl, 0, sizeof (impl));
  impl.size = 2;

  memset (&cimpl, 0, sizeof (impl));
  cimpl.size = 2;
#endif

#ifdef VISCORES
  fviscores = popen (
    "/usr/bin/gnuplot -background black"
    " -xrm 'gnuplot*textColor:white'"
    " -xrm 'gnuplot*borderColor:white'"
    " -xrm 'gnuplot*axisColor:white'"
    , "w");
  fprintf (fviscores, "unset key\n");
  // fprintf (fviscores, "set log y\n");
  fflush (fviscores);
  system ("rm -rf /tmp/picosat-viscores");
  system ("mkdir /tmp/picosat-viscores");
  system ("mkdir /tmp/picosat-viscores/data");
#ifdef WRITEGIF
  system ("mkdir /tmp/picosat-viscores/gif");
  fprintf (fviscores,
           "set terminal gif giant animate opt size 1024,768 x000000 xffffff"
	   "\n");

  fprintf (fviscores, 
           "set output \"/tmp/picosat-viscores/gif/animated.gif\"\n");
#endif
#endif
  defaultphase = JWLPHASE;
  state = READY;
  last_sat_call_result = 0;
}

static size_t
bytes_clause (unsigned size, unsigned learned)
{
  size_t res;

  res = sizeof (Cls);
  res += size * sizeof (Lit *);
  res -= 2 * sizeof (Lit *);

  if (learned && size > 2)
    res += sizeof (Act);	/* add activity */

#ifdef TRACE
  if (trace)
    res += sizeof (Trd);	/* add trace data */
#endif

  return res;
}

static Cls *
new_clause (unsigned size, unsigned learned)
{
  size_t bytes;
  void * tmp;
#ifdef TRACE
  Trd *trd;
#endif
  Cls *res;

  bytes = bytes_clause (size, learned);
  tmp = new (bytes);

#ifdef TRACE
  if (trace)
    {
      trd = tmp;

      if (learned)
	trd->idx = LIDX2IDX (lhead - lclauses);
      else
	trd->idx = OIDX2IDX (ohead - oclauses);

      res = trd->cls;
    }
  else
#endif
    res = tmp;

  res->size = size;
  res->learned = learned;

  res->collect = 0;
#ifndef NDEBUG
  res->connected = 0;
#endif
  res->locked = 0;
  res->used = 0;
#ifdef TRACE
  res->core = 0;
  res->collected = 0;
#endif

  if (learned && size > 2)
    *CLS2ACT (res) = cinc;

  return res;
}

static void
delete_clause (Cls * cls)
{
  size_t bytes;
#ifdef TRACE
  Trd *trd;
#endif

  bytes = bytes_clause (cls->size, cls->learned);

#ifdef TRACE
  if (trace)
    {
      trd = CLS2TRD (cls);
      delete (trd, bytes);
    }
  else
#endif
    delete (cls, bytes);
}

static void
delete_clauses (void)
{
  Cls **p;
  for (p = SOC; p != EOC; p = NXC (p))
    if (*p)
      delete_clause (*p);

  DELETEN (oclauses, eoo - oclauses);
  DELETEN (lclauses, eol - lclauses);

  ohead = eoo = lhead = eol = 0;
}

#ifdef TRACE

static void
delete_zhain (Zhn * zhain)
{
  const Znt *p, *znt;

  assert (zhain);

  znt = zhain->znt;
  for (p = znt; *p; p++)
    ;

  delete (zhain, sizeof (Zhn) + (p - znt) + 1);
}

static void
delete_zhains (void)
{
  Zhn **p, *z;
  for (p = zhains; p < zhead; p++)
    if ((z = *p))
      delete_zhain (z);

  DELETEN (zhains, eoz - zhains);
  eoz = zhead = 0;
}

#endif

#ifdef NO_BINARY_CLAUSES
static void
lrelease (Ltk * stk)
{
  if (stk->start)
    DELETEN (stk->start, (1 << (stk->ldsize)));
  memset (stk, 0, sizeof (*stk));
}
#endif

#ifndef NADC

static unsigned
llength (Lit ** a)
{
  Lit ** p;
  for (p = a; *p; p++)
    ;
  return p - a;
}

static void
resetadoconflict (void)
{
  assert (adoconflict);
  delete_clause (adoconflict);
  adoconflict = 0;
}

static void
reset_ados (void)
{
  Lit *** p;

  for (p = ados; p < hados; p++)
    DELETEN (*p, llength (*p) + 1);

  DELETEN (ados, eados - ados);
  hados = eados = 0;

  DELETEN (adotab, szadotab);
  szadotab = nadotab = 0;

  if (adoconflict)
    resetadoconflict ();

  adoconflicts = 0;
  adoconflictlimit = UINT_MAX;
  adodisabled = 0;
}

#endif

static void
reset (void)
{
  ABORTIF (state == RESET, "API usage: reset without initialization");

  delete_clauses ();
#ifdef TRACE
  delete_zhains ();
#endif
#ifdef NO_BINARY_CLAUSES
  implvalid = 0;
  cimplvalid = 0;
  {
    unsigned i;
    for (i = 2; i <= 2 * max_var + 1; i++)
      lrelease (impls + i);
  }
#endif
#ifndef NADC
  reset_ados ();
#endif
#ifndef NFL
  DELETEN (saved, saved_size);
  saved_size = 0;
#endif
  DELETEN (htps, 2 * size_vars);
#ifndef NDSC
  DELETEN (dhtps, 2 * size_vars);
#endif
  DELETEN (impls, 2 * size_vars);
  DELETEN (lits, 2 * size_vars);
  DELETEN (jwh, 2 * size_vars);
  DELETEN (vars, size_vars);
  DELETEN (rnks, size_vars);

  DELETEN (trail, eot - trail);
  trail = ttail = ttail2 = thead = eot = 0;
#ifndef NADC
  ttailado = 0;
#endif

  DELETEN (heap, eoh - heap);
  heap = hhead = eoh = 0;

  DELETEN (als, eoals - als);
  als = eoals = alshead = alstail = 0;
  extracted_all_failed_assumptions = 0;
  failed_assumption = 0;
  adecidelevel = 0;
  DELETEN (cls, eocls - cls);
  cls = eocls = clshead = 0;
  DELETEN (rils, eorils - rils);
  rils = eorils = rilshead = 0;
  DELETEN (cils, eocils - cils);
  cils = eocils = cilshead = 0;
  DELETEN (fals, eofals - fals);
  fals = eofals = falshead = 0;
  DELETEN (mass, szmass);
  szmass = 0;
  mass = 0;
  DELETEN (mssass, szmssass);
  szmssass = 0;
  mssass = 0;
  DELETEN (mcsass, szmcsass);
  nmcsass = szmcsass = 0;
  mcsass = 0;
  DELETEN (humus, szhumus);
  szhumus = 0;
  humus = 0;

  size_vars = 0;
  max_var = 0;

  mtcls = 0;
#ifdef TRACE
  ocore = -1;
#endif
  conflict = 0;

  DELETEN (added, eoa - added);
  eoa = ahead = 0;

  DELETEN (marked, eom - marked);
  eom = mhead = 0;

  DELETEN (dfs, eod - dfs);
  eod = dhead = 0;

  DELETEN (resolved, eor - resolved);
  eor = rhead = 0;

  DELETEN (buffer, eob - buffer);
  eob = bhead = 0;

  DELETEN (indices, eoi - indices);
  eoi = ihead = 0;

  DELETEN (soclauses, eoso - soclauses);
  soclauses = eoso = sohead = 0;
  saveorig = partial = 0;

  delete_prefix ();

  delete (rline[0], szrline);
  delete (rline[1], szrline);
  rline[0] = rline[1] = 0;
  szrline = rcount = 0;
  assert (getenv ("LEAK") || !current_bytes);	/* found leak if failing */
  max_bytes = 0;
  recycled = 0;
  current_bytes = 0;

  lrestart = 0;
  lreduce = 0;
  lastreduceconflicts = 0;
  llocked = 0;
  lsimplify = 0;
  fsimplify = 0;

  seconds = 0;
  flseconds = 0;
  entered = 0;
  nentered = 0;
  measurealltimeinlib = 0;

  levelsum = 0.0;
  calls = 0;
  decisions = 0;
  restarts = 0;
  simps = 0;
  iterations = 0;
  reports = 0;
  lastrheader = -2;
  fixed = 0;
#ifndef NFL
  failedlits = 0;
  simplifying = 0;
  fllimit = 0;
#ifdef STATS
  efailedlits = ifailedlits = 0;
  fltried = flskipped = floopsed = 0;
  flcalls = flrounds = 0;
  flprops = 0;
#endif
#endif
  propagations = 0;
  contexts = 0;
  internals = 0;
  conflicts = 0;
  noclauses = 0;
  oadded = 0;
  lladded = 0;
  loadded = 0;
  olits = 0;
  nlclauses = 0;
  ladded = 0;
  addedclauses = 0;
  llits = 0;
  out = 0;
#ifdef TRACE
  trace = 0;
#endif
  rup = 0;
  rupstarted = 0;
  rupclauses = 0;
  rupvariables = 0;
  level = 0;

  reductions = 0;

  vused = 0;
  llitsadded = 0;
  visits = 0;
#ifdef STATS
  loused = 0;
  llused = 0;
  bvisits = 0;
  tvisits = 0;
  lvisits = 0;
  othertrue = 0;
  othertrue2 = 0;
  othertruel = 0;
  othertrue2u = 0;
  othertruelu = 0;
  ltraversals = 0;
  traversals = 0;
#ifndef NO_BINARY_CLAUSES
  antecedents = 0;
#endif
  znts = 0;
  uips = 0;
  assumptions = 0;
  rdecisions = 0;
  sdecisions = 0;
  srecycled = 0;
  rrecycled = 0;
#endif
  minimizedllits = 0;
  nonminimizedllits = 0;
  state = RESET;
  srng = 0;

  saved_flips = 0;
  saved_max_var = 0;
  min_flipped = UINT_MAX;

  flips = 0;
#ifdef STATS
  forced = 0;
  assignments = 0;
#endif

  sdflips = 0;
  defaultphase = JWLPHASE;

#ifdef STATS
  staticphasedecisions = 0;
  inclreduces = 0;
  skippedrestarts = 0;
#endif

  emgr = 0;
  enew = 0;
  eresize = 0;
  edelete = 0;
#ifdef VISCORES
  pclose (fviscores);
  fviscores = 0;
#endif
}

inline static void
tpush (Lit * lit)
{
  assert (lits < lit && lit <= lits + 2* max_var + 1);
  if (thead == eot)
    {
      unsigned ttail2count = ttail2 - trail;
      unsigned ttailcount = ttail - trail;
#ifndef NADC
      unsigned ttailadocount = ttailado - trail;
#endif
      ENLARGE (trail, thead, eot);
      ttail = trail + ttailcount;
      ttail2 = trail + ttail2count;
#ifndef NADC
      ttailado = trail + ttailadocount;
#endif
    }

  *thead++ = lit;
}

static void
assign_reason (Var * v, Cls * reason)
{
#ifdef NO_BINARY_CLAUSES
  assert (reason != &impl);
#endif
  v->reason = reason;
}

static void
assign_phase (Lit * lit)
{
  unsigned new_phase, idx;
  Var * v = LIT2VAR (lit);

#ifndef NFL
  /* In 'simplifying' mode we only need to keep 'min_flipped' up to date if
   * we force assignments on the top level.   The other assignments will be
   * undone and thus we can keep the old saved value of the phase.
   */
  if (!level || !simplifying)
#endif
    {
      new_phase = (LIT2SGN (lit) > 0);

      if (v->assigned)
	{
	  sdflips -= sdflips/FFLIPPED;

	  if (new_phase != v->phase)
	    {
	      assert (FFLIPPEDPREC >= FFLIPPED);
	      sdflips += FFLIPPEDPREC / FFLIPPED;
	      flips++;

	      idx = LIT2IDX (lit);
	      if (idx < min_flipped)
		min_flipped = idx;

	      NOLOG (fprintf (out, "%sflipped %d\n", prefix, LIT2INT (lit)));
	    }
	}

      v->phase = new_phase;
      v->assigned = 1;
    }

  lit->val = TRUE;
  NOTLIT (lit)->val = FALSE;
}

inline static void
assign (Lit * lit, Cls * reason)
{
  Var * v = LIT2VAR (lit);
  assert (lit->val == UNDEF);
#ifdef STATS
  assignments++;
#endif
  v->level = level;
  assign_phase (lit);
  assign_reason (v, reason);
  tpush (lit);
}

inline static int
cmp_added (Lit * k, Lit * l)
{
  Val a = k->val, b = l->val;
  Var *u, *v;
  int res;

  if (a == UNDEF && b != UNDEF)
    return -1;

  if (a != UNDEF && b == UNDEF)
    return 1;

  u = LIT2VAR (k);
  v = LIT2VAR (l);

  if (a != UNDEF)
    {
      assert (b != UNDEF);
      res = v->level - u->level;
      if (res)
	return res;		/* larger level first */
    }

  res = cmpflt (VAR2RNK (u)->score, VAR2RNK (v)->score);
  if (res)
    return res;			/* smaller activity first */

  return u - v;			/* smaller index first */
}

static void
sorttwolits (Lit ** v)
{
  Lit * a = v[0], * b = v[1];

  assert (a != b);

  if (a < b)
    return;

  v[0] = b;
  v[1] = a;
}

inline static void
sortlits (Lit ** v, unsigned size)
{
  if (size == 2)
    sorttwolits (v);	/* same order with and with out 'NO_BINARY_CLAUSES' */
  else
    SORT (Lit *, cmp_added, v, size);
}

#ifdef NO_BINARY_CLAUSES
static Cls *
setimpl (Lit * a, Lit * b)
{
  assert (!implvalid);
  assert (impl.size == 2);

  impl.lits[0] = a;
  impl.lits[1] = b;

  sorttwolits (impl.lits);
  implvalid = 1;

  return &impl;
}

static void
resetimpl (void)
{
  implvalid = 0;
}

static Cls *
setcimpl (Lit * a, Lit * b)
{
  assert (!cimplvalid);
  assert (cimpl.size == 2);

  cimpl.lits[0] = a;
  cimpl.lits[1] = b;

  sorttwolits (cimpl.lits);
  cimplvalid = 1;

  return &cimpl;
}

static void
resetcimpl (void)
{
  assert (cimplvalid);
  cimplvalid = 0;
}

#endif

static int
cmp_ptr (void *l, void *k)
{
  return ((char*)l) - (char*)k;		/* arbitrarily already reverse */
}

static int
cmp_rnk (Rnk * r, Rnk * s)
{
  if (!r->moreimportant && s->moreimportant)
    return -1;

  if (r->moreimportant && !s->moreimportant)
    return 1;

  if (!r->lessimportant && s->lessimportant)
    return 1;

  if (r->lessimportant && !s->lessimportant)
    return -1;

  if (r->score < s->score)
    return -1;

  if (r->score > s->score)
    return 1;

  return -cmp_ptr (r, s);
}

static void
hup (Rnk * v)
{
  int upos, vpos;
  Rnk *u;

#ifndef NFL
  assert (!simplifying);
#endif

  vpos = v->pos;

  assert (0 < vpos);
  assert (vpos < hhead - heap);
  assert (heap[vpos] == v);

  while (vpos > 1)
    {
      upos = vpos / 2;

      u = heap[upos];

      if (cmp_rnk (u, v) > 0)
	break;

      heap[vpos] = u;
      u->pos = vpos;

      vpos = upos;
    }

  heap[vpos] = v;
  v->pos = vpos;
}

static Cls *add_simplified_clause (int learned);

inline static void
add_antecedent (Cls * c)
{
  assert (c);

#ifdef NO_BINARY_CLAUSES
  if (ISLITREASON (c))
    return;

  if (c == &impl)
    return;
#else
#ifdef STATS
  antecedents++;
#endif
#endif
  if (rhead == eor)
    ENLARGE (resolved, rhead, eor);

  assert (rhead < eor);
  *rhead++ = c;
}

#ifdef TRACE

#ifdef NO_BINARY_CLAUSES
#error "can not combine TRACE and NO_BINARY_CLAUSES"
#endif

#endif /* TRACE */

static void
add_lit (Lit * lit)
{
  assert (lit);

  if (ahead == eoa)
    ENLARGE (added, ahead, eoa);

  *ahead++ = lit;
}

static void
push_var_as_marked (Var * v)
{
  if (mhead == eom)
    ENLARGE (marked, mhead, eom);

  *mhead++ = v;
}

static void
mark_var (Var * v)
{
  assert (!v->mark);
  v->mark = 1;
  push_var_as_marked (v);
}

#ifdef NO_BINARY_CLAUSES

static Cls *
impl2reason (Lit * lit)
{
  Lit * other;
  Cls * res;
  other = impl.lits[0];
  if (lit == other)
    other = impl.lits[1];
  assert (other->val == FALSE);
  res = LIT2REASON (NOTLIT (other));
  resetimpl ();
  return res;
}

#endif

/* Whenever we have a top level derived unit we really should derive a unit
 * clause otherwise the resolutions in 'add_simplified_clause' become
 * incorrect.
 */
static Cls *
resolve_top_level_unit (Lit * lit, Cls * reason)
{
  unsigned count_resolved;
  Lit **p, **eol, *other;
  Var *u, *v;

  assert (rhead == resolved);
  assert (ahead == added);

  add_lit (lit);
  add_antecedent (reason);
  count_resolved = 1;
  v = LIT2VAR (lit);

  eol = end_of_lits (reason);
  for (p = reason->lits; p < eol; p++)
    {
      other = *p;
      u = LIT2VAR (other);
      if (u == v)
	continue;

      add_antecedent (u->reason);
      count_resolved++;
    }

  /* Some of the literals could be assumptions.  If at least one
   * variable is not an assumption, we should resolve.
   */
  if (count_resolved >= 2)
    {
#ifdef NO_BINARY_CLAUSES
      if (reason == &impl)
	resetimpl ();
#endif
      reason = add_simplified_clause (1);
#ifdef NO_BINARY_CLAUSES
      if (reason->size == 2)
	{
	  assert (reason == &impl);
	  reason = impl2reason (lit);
	}
#endif
      assign_reason (v, reason);
    }
  else
    {
      ahead = added;
      rhead = resolved;
    }

  return reason;
}

static void
fixvar (Var * v)
{
  Rnk * r;

  assert (VAR2LIT (v) != UNDEF);
  assert (!v->level);

  fixed++;

  r = VAR2RNK (v);
  r->score = INFFLT;

#ifndef NFL
  if (simplifying)
    return;
#endif

  if (!r->pos)
    return;

  hup (r);
}

static void
use_var (Var * v)
{
  if (v->used)
    return;

  v->used = 1;
  vused++;
}

static void
assign_forced (Lit * lit, Cls * reason)
{
  Var *v;

  assert (reason);
  assert (lit->val == UNDEF);

#ifdef STATS
  forced++;
#endif
  assign (lit, reason);

#ifdef NO_BINARY_CLAUSES
  assert (reason != &impl);
  if (ISLITREASON (reason)) 
    {
      reason = setimpl (lit, NOTLIT (REASON2LIT (reason)));
      assert (reason);
    }
#endif
  LOG (fprintf (out,
                "%sassign %d at level %d by ",
                prefix, LIT2INT (lit), level);
       dumpclsnl (reason));

  v = LIT2VAR (lit);
  if (!level)
    use_var (v);

  if (!level && reason->size > 1)
    {
      reason = resolve_top_level_unit (lit, reason);
      assert (reason);
    }

#ifdef NO_BINARY_CLAUSES
  if (ISLITREASON (reason) || reason == &impl)
    {
      /* DO NOTHING */
    }
  else
#endif
    {
      assert (!reason->locked);
      reason->locked = 1;
      if (reason->learned && reason->size > 2)
	llocked++;
    }

#ifdef NO_BINARY_CLAUSES
  if (reason == &impl)
    resetimpl ();
#endif

  if (!level)
    fixvar (v);
}

#ifdef NO_BINARY_CLAUSES

static void
lpush (Lit * lit, Cls * cls)
{
  int pos = (cls->lits[0] == lit);
  Ltk * s = LIT2IMPLS (lit);
  unsigned oldsize, newsize;

  assert (cls->size == 2);

  if (!s->start)
    {
      assert (!s->count);
      assert (!s->ldsize);
      NEWN (s->start, 1);
    }
  else 
    {
      oldsize = (1 << (s->ldsize));
      assert (s->count <= oldsize);
      if (s->count == oldsize)
	{
	  newsize = 2 * oldsize;
	  RESIZEN (s->start, oldsize, newsize);
	  s->ldsize++;
	}
    }

  s->start[s->count++] = cls->lits[pos];
}

#endif

static void
connect_head_tail (Lit * lit, Cls * cls)
{
  Cls ** s;
  assert (cls->size >= 1);
  if (cls->size == 2)
    {
#ifdef NO_BINARY_CLAUSES
      lpush (lit, cls);
      return;
#else
      s = LIT2IMPLS (lit);
#endif
    }
  else
    s = LIT2HTPS (lit);

  if (cls->lits[0] != lit)
    {
      assert (cls->size >= 2);
      assert (cls->lits[1] == lit);
      cls->next[1] = *s;
    }
  else
    cls->next[0] = *s;

  *s = cls;
}

#ifdef TRACE
static void
zpush (Zhn * zhain)
{
  assert (trace);

  if (zhead == eoz)
    ENLARGE (zhains, zhead, eoz);

  *zhead++ = zhain;
}

static int
cmp_resolved (Cls * c, Cls * d)
{
  assert (trace);

  return CLS2IDX (c) - CLS2IDX (d);
}

static void
bpushc (unsigned char ch)
{
  if (bhead == eob)
    ENLARGE (buffer, bhead, eob);

  *bhead++ = ch;
}

static void
bpushu (unsigned u)
{
  while (u & ~0x7f)
    {
      bpushc (u | 0x80);
      u >>= 7;
    }

  bpushc (u);
}

static void
bpushd (unsigned prev, unsigned this)
{
  unsigned delta;
  assert (prev < this);
  delta = this - prev;
  bpushu (delta);
}

static void
add_zhain (void)
{
  unsigned prev, this, count, rcount;
  Cls **p, *c;
  Zhn *res;

  assert (trace);
  assert (bhead == buffer);
  assert (rhead > resolved);

  rcount = rhead - resolved;
  SORT (Cls *, cmp_resolved, resolved, rcount);

  prev = 0;
  for (p = resolved; p < rhead; p++)
    {
      c = *p;
      this = CLS2TRD (c)->idx;
      bpushd (prev, this);
      prev = this;
    }
  bpushc (0);

  count = bhead - buffer;

  res = new (sizeof (Zhn) + count);
  res->core = 0;
  res->ref = 0;
  memcpy (res->znt, buffer, count);

  bhead = buffer;
#ifdef STATS
  znts += count - 1;
#endif
  zpush (res);
}

#endif

static void
add_resolved (int learned)
{
#if defined(STATS) || defined(TRACE)
  Cls **p, *c;

  for (p = resolved; p < rhead; p++)
    {
      c = *p;
      if (c->used)
	continue;

      c->used = 1;

      if (c->size <= 2)
	continue;

#ifdef STATS
      if (c->learned)
	llused++;
      else
	loused++;
#endif
    }
#endif

#ifdef TRACE
  if (learned && trace)
    add_zhain ();
#else
  (void) learned;
#endif
  rhead = resolved;
}

static void
incjwh (Cls * cls)
{
  Lit **p, *lit, ** eol;
  Flt * f, inc, sum;
  unsigned size = 0;
  Var * v;
  Val val;

  eol = end_of_lits (cls);

  for (p = cls->lits; p < eol; p++)
    {
      lit = *p;
      val = lit->val;

      if (val && level > 0)
	{
	  v = LIT2VAR (lit);
	  if (v->level > 0)
	    val = UNDEF;
	}

      if (val == TRUE)
	return;

      if (val != FALSE)
	size++;
    }

  inc = base2flt (1, -size);

  for (p = cls->lits; p < eol; p++)
    {
      lit = *p;
      f = LIT2JWH (lit);
      sum = addflt (*f, inc);
      *f = sum;
    }
}

static void
write_rup_header (FILE * file)
{
  char line[80];
  int i;

  sprintf (line, "%%RUPD32 %u %u", rupvariables, rupclauses);

  fputs (line, file);
  for (i = 255 - strlen (line); i >= 0; i--)
    fputc (' ', file);

  fputc ('\n', file);
  fflush (file);
}

static Cls *
add_simplified_clause (int learned)
{
  unsigned num_true, num_undef, num_false, size, count_resolved;
  Lit **p, **q, *lit, ** end;
  Cls *res, * reason;
  int reentered;
  Val val;
  Var *v;
#if !defined(NDEBUG) && defined(TRACE)
  unsigned idx;
#endif

  reentered = 0;

REENTER:

  size = ahead - added;

  add_resolved (learned);

  if (learned)
    {
      ladded++;
      llitsadded += size;
      if (size > 2)
	{
	  lladded++;
	  nlclauses++;
	  llits += size;
	}
    }
  else
    {
      oadded++;
      if (size > 2)
	{
	  loadded++;
	  noclauses++;
	  olits += size;
	}
    }

  addedclauses++;
  assert (addedclauses == ladded + oadded);

#ifdef NO_BINARY_CLAUSES
  if (size == 2)
    res = setimpl (added[0], added[1]);
  else
#endif
    {
      sortlits (added, size); 

      if (learned)
	{
	  if (lhead == eol)
	    {
	      ENLARGE (lclauses, lhead, eol);

	      /* A very difficult to find bug, which only occurs if the
	       * learned clauses stack is immediately allocated before the
	       * original clauses stack without padding.  In this case, we
	       * have 'SOC == EOC', which terminates all loops using the
	       * idiom 'for (p = SOC; p != EOC; p = NXC(p))' immediately.
	       * Unfortunately this occurred in 'fix_clause_lits' after
	       * using a recent version of the memory allocator of 'Google'
	       * perftools in the context of one large benchmark for 
	       * our SMT solver 'Boolector'.
	       */
	      if (eol == oclauses)
		ENLARGE (lclauses, lhead, eol);
	    }

#if !defined(NDEBUG) && defined(TRACE)
	  idx = LIDX2IDX (lhead - lclauses);
#endif
	}
      else
	{
	  if (ohead == eoo)
	    {
	      ENLARGE (oclauses, ohead, eoo);
	      if (eol == oclauses)
		ENLARGE (oclauses, ohead, eoo);	/* ditto */
	    }

#if !defined(NDEBUG) && defined(TRACE)
	  idx = OIDX2IDX (ohead - oclauses);
#endif
	}

      assert (eol != oclauses);			/* ditto */

      res = new_clause (size, learned);

#if !defined(NDEBUG) && defined(TRACE)
      if (trace)
	assert (CLS2IDX (res) == idx);
#endif
      if (learned)
	*lhead++ = res;
      else
	*ohead++ = res;

#if !defined(NDEBUG) && defined(TRACE)
      if (trace && learned)
	assert (zhead - zhains == lhead - lclauses);
#endif
      assert (lhead != oclauses);		/* ditto */
    }

  if (learned && rup)
    {
      if (!rupstarted)
	{
	  write_rup_header (rup);
	  rupstarted = 1;
	}
    }

  num_true = num_undef = num_false = 0;

  q = res->lits;
  for (p = added; p < ahead; p++)
    {
      lit = *p;
      *q++ = lit;

      if (learned && rup)
	fprintf (rup, "%d ", LIT2INT (lit));

      val = lit->val;

      num_true += (val == TRUE);
      num_undef += (val == UNDEF);
      num_false += (val == FALSE);
    }
  assert (num_false + num_true + num_undef == size);

  if (learned && rup)
    fputs ("0\n", rup);

  ahead = added;		/* reset */

  if (!reentered)				// TODO merge
  if (size > 0)
    {
      assert (size <= 2 || !reentered);		// TODO remove
      connect_head_tail (res->lits[0], res);
      if (size > 1)
	connect_head_tail (res->lits[1], res);
    }

  if (size == 0)
    {
      if (!mtcls)
	mtcls = res;
    }

#ifdef NO_BINARY_CLAUSES
  if (size != 2)
#endif
#ifndef NDEBUG
    res->connected = 1;
#endif

  LOG (fprintf (out, "%s%s ", prefix, learned ? "learned" : "original");
       dumpclsnl (res));

  /* Shrink clause by resolving it against top level assignments.
   */
  if (!level && num_false > 0)
    {
      assert (ahead == added);
      assert (rhead == resolved);

      count_resolved = 1;
      add_antecedent (res);

      end = end_of_lits (res);
      for (p = res->lits; p < end; p++)
	{
	  lit = *p;
	  v = LIT2VAR (lit);
	  use_var (v);

	  if (lit->val == FALSE)
	    {
	      add_antecedent (v->reason);
	      count_resolved++;
	    }
	  else
	    add_lit (lit);
	}

      assert (count_resolved >= 2);

      learned = 1;
#ifdef NO_BINARY_CLAUSES
      if (res == &impl)
	resetimpl ();
#endif
      reentered = 1;
      goto REENTER;		/* and return simplified clause */
    }

  if (!num_true && num_undef == 1)	/* unit clause */
    {
      lit = 0;
      for (p = res->lits; p < res->lits + size; p++)
	{
	  if ((*p)->val == UNDEF)
	    lit = *p;

	  v = LIT2VAR (*p);
	  use_var (v);
	}
      assert (lit);

      reason = res;
#ifdef NO_BINARY_CLAUSES
      if (size == 2)
        {
	  Lit * other = res->lits[0];
	  if (other == lit)
	    other = res->lits[1];

	  assert (other->val == FALSE);
	  reason = LIT2REASON (NOTLIT (other));
	}
#endif
      assign_forced (lit, reason);
      num_true++;
    }

  if (num_false == size && !conflict)
    {
#ifdef NO_BINARY_CLAUSES
      if (res == &impl)
	conflict = setcimpl (res->lits[0], res->lits[1]);
      else
#endif
      conflict = res;
    }

  if (!learned && !num_true && num_undef)
    incjwh (res);

#ifdef NO_BINARY_CLAUSES
  if (res == &impl)
    resetimpl ();
#endif
  return res;
}

static int
trivial_clause (void)
{
  Lit **p, **q, *prev;
  Var *v;

  SORT (Lit *, cmp_ptr, added,  ahead - added);

  prev = 0;
  q = added;
  for (p = q; p < ahead; p++)
    {
      Lit *this = *p;

      v = LIT2VAR (this);

      if (prev == this)		/* skip repeated literals */
	continue;

      /* Top level satisfied ? 
       */
      if (this->val == TRUE && !v->level)
	 return 1;

      if (prev == NOTLIT (this))/* found pair of dual literals */
	return 1;

      *q++ = prev = this;
    }

  ahead = q;			/* shrink */

  return 0;
}

static void
simplify_and_add_original_clause (void)
{
#ifdef NO_BINARY_CLAUSES
  Cls * c;
#endif
  if (trivial_clause ())
    {
      ahead = added;

      if (ohead == eoo)
	ENLARGE (oclauses, ohead, eoo);

      *ohead++ = 0;

      addedclauses++;
      oadded++;
    }
  else
    {
      if (cls != clshead)
	add_lit (NOTLIT (clshead[-1]));

#ifdef NO_BINARY_CLAUSES
      c = 
#endif
      add_simplified_clause (0);
#ifdef NO_BINARY_CLAUSES
      if (c == &impl) assert (!implvalid);
#endif
    }
}

#ifndef NADC

static void
add_ado (void)
{
  unsigned len = ahead - added;
  Lit ** ado, ** p, ** q, *lit;
  Var * v, * u;

#ifdef TRACE
  assert (!trace);
#endif

  ABORTIF (ados < hados && llength (ados[0]) != len,
           "internal: non matching all different constraint object lengths");

  if (hados == eados)
    ENLARGE (ados, hados, eados);

  NEWN (ado, len + 1);
  *hados++ = ado;

  p = added;
  q = ado;
  u = 0;
  while (p < ahead)
    {
      lit = *p++;
      v = LIT2VAR (lit);
      ABORTIF (v->inado, 
               "internal: variable in multiple all different objects");
      v->inado = ado;
      if (!u && !lit->val)
	u = v;
      *q++ = lit;
    }

  assert (q == ado + len);
  *q++ = 0;

  /* TODO simply do a conflict test as in propado */

  ABORTIF (!u,
    "internal: "
    "adding fully instantiated all different object not implemented yet");

  assert (u);
  assert (u->inado == ado);
  assert (!u->ado);
  u->ado = ado;

  ahead = added;
}

#endif

static void
hdown (Rnk * r)
{
  unsigned end, rpos, cpos, opos;
  Rnk *child, *other;

  assert (r->pos > 0);
  assert (heap[r->pos] == r);

  end = hhead - heap;
  rpos = r->pos;

  for (;;)
    {
      cpos = 2 * rpos;
      if (cpos >= end)
	break;

      opos = cpos + 1;
      child = heap[cpos];

      if (cmp_rnk (r, child) < 0)
	{
	  if (opos < end)
	    {
	      other = heap[opos];

	      if (cmp_rnk (child, other) < 0)
		{
		  child = other;
		  cpos = opos;
		}
	    }
	}
      else if (opos < end)
	{
	  child = heap[opos];

	  if (cmp_rnk (r, child) >= 0)
	    break;

	  cpos = opos;
	}
      else
	break;

      heap[rpos] = child;
      child->pos = rpos;
      rpos = cpos;
    }

  r->pos = rpos;
  heap[rpos] = r;
}

static Rnk *
htop (void)
{
  assert (hhead > heap + 1);
  return heap[1];
}

static Rnk *
hpop (void)
{
  Rnk *res, *last;
  unsigned end;

  assert (hhead > heap + 1);

  res = heap[1];
  res->pos = 0;

  end = --hhead - heap;
  if (end == 1)
    return res;

  last = heap[end];

  heap[last->pos = 1] = last;
  hdown (last);

  return res;
}

inline static void
hpush (Rnk * r)
{
  assert (!r->pos);

  if (hhead == eoh)
    ENLARGE (heap, hhead, eoh);

  r->pos = hhead++ - heap;
  heap[r->pos] = r;
  hup (r);
}

static void
fix_trail_lits (long delta)
{
  Lit **p;
  for (p = trail; p < thead; p++)
    *p += delta;
}

#ifdef NO_BINARY_CLAUSES
static void
fix_impl_lits (long delta)
{
  Ltk * s;
  Lit ** p;

  for (s = impls + 2; s < impls + 2 * max_var; s++)
    for (p = s->start; p < s->start + s->count; p++)
      *p += delta;
}
#endif

static void
fix_clause_lits (long delta)
{
  Cls **p, *clause;
  Lit **q, *lit, **eol;

  for (p = SOC; p != EOC; p = NXC (p))
    {
      clause = *p;
      if (!clause)
	continue;

      q = clause->lits;
      eol = end_of_lits (clause);
      while (q < eol)
	{
	  assert (q - clause->lits <= (int) clause->size);
	  lit = *q;
	  lit += delta;
	  *q++ = lit;
	}
    }
}

static void
fix_added_lits (long delta)
{
  Lit **p;
  for (p = added; p < ahead; p++)
    *p += delta;
}

static void
fix_assumed_lits (long delta)
{
  Lit **p;
  for (p = als; p < alshead; p++)
    *p += delta;
}

static void
fix_cls_lits (long delta)
{
  Lit **p;
  for (p = cls; p < clshead; p++)
    *p += delta;
}

static void
fix_heap_rnks (long delta)
{
  Rnk **p;

  for (p = heap + 1; p < hhead; p++)
    *p += delta;
}

#ifndef NADC

static void
fix_ado (long delta, Lit ** ado)
{
  Lit ** p;
  for (p = ado; *p; p++)
    *p += delta;
}

static void
fix_ados (long delta)
{
  Lit *** p;

  for (p = ados; p < hados; p++)
    fix_ado (delta, *p);
}

#endif

static void
enlarge (unsigned new_size_vars)
{
  long rnks_delta, lits_delta;
  Lit *old_lits = lits;
  Rnk *old_rnks = rnks;

  RESIZEN (lits, 2 * size_vars, 2 * new_size_vars);
  RESIZEN (jwh, 2 * size_vars, 2 * new_size_vars);
  RESIZEN (htps, 2 * size_vars, 2 * new_size_vars);
#ifndef NDSC
  RESIZEN (dhtps, 2 * size_vars, 2 * new_size_vars);
#endif
  RESIZEN (impls, 2 * size_vars, 2 * new_size_vars);
  RESIZEN (vars, size_vars, new_size_vars);
  RESIZEN (rnks, size_vars, new_size_vars);

  lits_delta = lits - old_lits;
  rnks_delta = rnks - old_rnks;

  fix_trail_lits (lits_delta);
  fix_clause_lits (lits_delta);
  fix_added_lits (lits_delta);
  fix_assumed_lits (lits_delta);
  fix_cls_lits (lits_delta);
#ifdef NO_BINARY_CLAUSES
  fix_impl_lits (lits_delta);
#endif
#ifndef NADC
  fix_ados (lits_delta);
#endif
  fix_heap_rnks (rnks_delta);
  assert (mhead == marked);

  size_vars = new_size_vars;
}

static void
unassign (Lit * lit)
{
  Cls *reason;
  Var *v;
  Rnk *r;

  assert (lit->val == TRUE);

  LOG (fprintf (out, "%sunassign %d\n", prefix, LIT2INT (lit)));

  v = LIT2VAR (lit);
  reason = v->reason;

#ifdef NO_BINARY_CLAUSES
  assert (reason != &impl);
  if (ISLITREASON (reason))
    {
      /* DO NOTHING */
    }
  else
#endif
  if (reason)
    {
      assert (reason->locked);
      reason->locked = 0;
      if (reason->learned && reason->size > 2)
	{
	  assert (llocked > 0);
	  llocked--;
	}
    }

  lit->val = UNDEF;
  NOTLIT (lit)->val = UNDEF;

  r = VAR2RNK (v);
  if (!r->pos)
    hpush (r);

#ifndef NDSC
  {
    Cls * p, * next, ** q;

    q = LIT2DHTPS (lit);
    p = *q;
    *q = 0;

    while (p)
      {
	Lit * other = p->lits[0];

	if (other == lit)
	  {
	    other = p->lits[1];
	    q = p->next + 1;
	  }
	else
	  {
	    assert (p->lits[1] == lit);
	    q = p->next;
	  }

	next = *q;
	*q = *LIT2HTPS (other);
	*LIT2HTPS (other) = p;
	p = next;
      }
  }
#endif

#ifndef NADC
  if (v->adotabpos)
    {
      assert (nadotab);
      assert (*v->adotabpos == v->ado);

      *v->adotabpos = 0;
      v->adotabpos = 0;

      nadotab--;
    }
#endif
}

static Cls *
var2reason (Var * var)
{
  Cls * res = var->reason;
#ifdef NO_BINARY_CLAUSES
  Lit * this, * other;
  if (ISLITREASON (res))
    {
      this = VAR2LIT (var);
      if (this->val == FALSE)
	this = NOTLIT (this);

      other = REASON2LIT (res);
      assert (other->val == TRUE);
      assert (this->val == TRUE);
      res = setimpl (NOTLIT (other), this);
    }
#endif
  return res;
}

static void
mark_clause_to_be_collected (Cls * cls)
{
  assert (!cls->collect);
  cls->collect = 1;
}

static void
undo (unsigned new_level)
{
  Lit *lit;
  Var *v;

  while (thead > trail)
    {
      lit = *--thead;
      v = LIT2VAR (lit);
      if (v->level == new_level)
	{
	  thead++;		/* fix pre decrement */
	  break;
	}

      unassign (lit);
    }

  level = new_level;
  ttail = thead;
  ttail2 = thead;
#ifndef NADC
  ttailado = thead;
#endif

#ifdef NO_BINARY_CLAUSES
  if (conflict == &cimpl)
    resetcimpl ();
#endif
#ifndef NADC
  if (conflict && conflict == adoconflict)
    resetadoconflict ();
#endif
  conflict = mtcls;
  if (level < adecidelevel)
    {
      assert (als < alshead);
      adecidelevel = 0;
      alstail = als;
    }
  LOG (fprintf (out, "%sback to level %u\n", prefix, level));
}

#ifndef NDEBUG

static int
clause_satisfied (Cls * cls)
{
  Lit **p, **eol, *lit;

  eol = end_of_lits (cls);
  for (p = cls->lits; p < eol; p++)
    {
      lit = *p;
      if (lit->val == TRUE)
	return 1;
    }

  return 0;
}

static void
original_clauses_satisfied (void)
{
  Cls **p, *cls;

  for (p = oclauses; p < ohead; p++)
    {
      cls = *p;

      if (!cls)
	continue;

      if (cls->learned)
	continue;

      assert (clause_satisfied (cls));
    }
}

static void
assumptions_satisfied (void)
{
  Lit *lit, ** p;

  for (p = als; p < alshead; p++)
    {
      lit = *p;
      assert (lit->val == TRUE);
    }
}

#endif

static void
sflush (void)
{
  double now = picosat_time_stamp ();
  double delta = now - entered;
  delta = (delta < 0) ? 0 : delta;
  seconds += delta;
  entered = now;
}

static double
mb (void)
{
  return current_bytes / (double) (1 << 20);
}

static double
avglevel (void)
{
  return decisions ? levelsum / decisions : 0.0;
}

static void
rheader (void)
{
  assert (lastrheader <= reports);

  if (lastrheader == reports)
    return;

  lastrheader = reports;

  fprintf (out, "%s\n", prefix);
  fprintf (out, "%s %s\n", prefix, rline[0]);
  fprintf (out, "%s %s\n", prefix, rline[1]);
  fprintf (out, "%s\n", prefix);
}

static unsigned
dynamic_flips_per_assignment_per_mille (void)
{
  assert (FFLIPPEDPREC >= 1000);
  return sdflips / (FFLIPPEDPREC / 1000);
}

#ifdef NLUBY

static int
high_agility (void)
{
  return dynamic_flips_per_assignment_per_mille () >= 200;
}

static int
very_high_agility (void)
{
  return dynamic_flips_per_assignment_per_mille () >= 250;
}

#else

static int
medium_agility (void)
{
  return dynamic_flips_per_assignment_per_mille () >= 230;
}

#endif

static void
relemdata (void)
{
  char *p;
  int x;

  if (reports < 0)
    {
      /* strip trailing white space 
       */
      for (x = 0; x <= 1; x++)
	{
	  p = rline[x] + strlen (rline[x]);
	  while (p-- > rline[x])
	    {
	      if (*p != ' ')
		break;

	      *p = 0;
	    }
	}

      rheader ();
    }
  else
    fputc ('\n', out);

  rcount = 0;
}

static void
relemhead (const char * name, int fp, double val)
{
  int x, y, len, size;
  const char *fmt;
  unsigned tmp, e;

  if (reports < 0)
    {
      x = rcount & 1;
      y = (rcount / 2) * 12 + x * 6;

      if (rcount == 1)
	sprintf (rline[1], "%6s", "");

      len = strlen (name);
      while (szrline <= len + y + 1)
	{
	  size = szrline ? 2 * szrline : 128;
	  rline[0] = resize (rline[0], szrline, size);
	  rline[1] = resize (rline[1], szrline, size);
	  szrline = size;
	}

      fmt = (len <= 6) ? "%6s%10s" : "%-10s%4s";
      sprintf (rline[x] + y, fmt, name, "");
    }
  else if (val < 0)
    {
      assert (fp);

      if (val > -100 && (tmp = val * 10.0 - 0.5) > -1000.0)
	{
	  fprintf (out, "-%4.1f ", -tmp / 10.0);
	}
      else
	{
	  tmp = -val / 10.0 + 0.5;
	  e = 1;
	  while (tmp >= 100)
	    {
	      tmp /= 10;
	      e++;
	    }

	  fprintf (out, "-%2ue%u ", tmp, e);
	}
    }
  else
    {
      if (fp && val < 1000 && (tmp = val * 10.0 + 0.5) < 10000)
	{
	  fprintf (out, "%5.1f ", tmp / 10.0);
	}
      else if (!fp && (tmp = val) < 100000)
	{
	  fprintf (out, "%5u ", tmp);
	}
      else
	{
	  tmp = val / 10.0 + 0.5;
	  e = 1;

	  while (tmp >= 1000)
	    {
	      tmp /= 10;
	      e++;
	    }

	  fprintf (out, "%3ue%u ", tmp, e);
	}
    }

  rcount++;
}

inline static void
relem (const char *name, int fp, double val)
{
  if (name)
    relemhead (name, fp, val);
  else
    relemdata ();
}

static unsigned
reduce_limit_on_lclauses (void)
{
  unsigned res = lreduce;
  res += llocked;
  return res;
}

static void
report (int level, char type)
{
  int rounds;

  if (verbosity < level)
    return;

  sflush ();

  if (!reports)
    reports = -1;

  for (rounds = (reports < 0) ? 2 : 1; rounds; rounds--)
    {
      if (reports >= 0)
	fprintf (out, "%s%c ", prefix, type);

      relem ("seconds", 1, seconds);
      relem ("level", 1, avglevel ());
      assert (fixed <=  max_var);
      relem ("variables", 0, max_var - fixed);
      relem ("used", 1, PERCENT (vused, max_var));
      relem ("original", 0, noclauses);
      relem ("conflicts", 0, conflicts);
      //relem ("decisions", 0, decisions);
      // relem ("conf/dec", 1, PERCENT(conflicts,decisions));
      // relem ("limit", 0, reduce_limit_on_lclauses ());
      relem ("learned", 0, nlclauses);
      // relem ("limit", 1, PERCENT (nlclauses, reduce_limit_on_lclauses ()));
      relem ("limit", 0, lreduce);
#ifdef STATS
      relem ("learning", 1, PERCENT (llused, lladded));
#endif
      relem ("agility", 1, dynamic_flips_per_assignment_per_mille () / 10.0);
      // relem ("original", 0, noclauses);
      relem ("MB", 1, mb ());
      // relem ("lladded", 0, lladded);
      // relem ("llused", 0, llused);

      relem (0, 0, 0);

      reports++;
    }

  /* Adapt this to the number of rows in your terminal.
   */
  #define ROWS 25

  if (reports % (ROWS - 3) == (ROWS - 4))
    rheader ();

  fflush (out);
}

static int
bcp_queue_is_empty (void)
{
  if (ttail != thead)
    return 0;

  if (ttail2 != thead)
    return 0;

#ifndef NADC
  if (ttailado != thead)
    return 0;
#endif

  return 1;
}

static int
satisfied (void)
{
  assert (!mtcls);
  assert (!failed_assumption);
  if (alstail < alshead)
    return 0;
  assert (!conflict);
  assert (bcp_queue_is_empty ());
  return thead == trail + max_var;	/* all assigned */
}

static void
vrescore (void)
{
  Rnk *p, *eor = rnks + max_var;
  for (p = rnks + 1; p <= eor; p++)
    if (p->score != INFFLT)
      p->score = mulflt (p->score, ilvinc);
  vinc = mulflt (vinc, ilvinc);;
#ifdef VISCORES
  nvinc = mulflt (nvinc, lscore);;
#endif
}

static void
inc_score (Var * v)
{
  Flt score;
  Rnk *r;

#ifndef NFL
  if (simplifying)
    return;
#endif

  if (!v->level)
    return;

  if (v->internal)
    return;

  r = VAR2RNK (v);
  score = r->score;

  assert (score != INFFLT);

  score = addflt (score, vinc);
  assert (score < INFFLT);
  r->score = score;
  if (r->pos > 0)
    hup (r);

  if (score > lscore)
    vrescore ();
}

static void
inc_activity (Cls * cls)
{
  Act *p;

  if (!cls->learned)
    return;

  if (cls->size <= 2)
    return;

  p = CLS2ACT (cls);
  *p = addflt (*p, cinc);
}

static unsigned
hashlevel (unsigned l)
{
  return 1u << (l & 31);
}

static void
push (Var * v)
{
  if (dhead == eod)
    ENLARGE (dfs, dhead, eod);

  *dhead++ = v;
}

static Var * 
pop (void)
{
  assert (dfs < dhead);
  return *--dhead;
}

static void
analyze (void)
{
  unsigned open, minlevel, siglevels, l, old, i, orig;
  Lit *this, *other, **p, **q, **eol;
  Var *v, *u, **m, *start, *uip;
  Cls *c;

  assert (conflict);

  assert (ahead == added);
  assert (mhead == marked);
  assert (rhead == resolved);

  /* First, search for First UIP variable and mark all resolved variables.
   * At the same time determine the minimum decision level involved.
   * Increase activities of resolved variables.
   */
  q = thead;
  open = 0;
  minlevel = level;
  siglevels = 0;
  uip = 0;

  c = conflict;

  for (;;)
    {
      add_antecedent (c);
      inc_activity (c);
      eol = end_of_lits (c);
      for (p = c->lits; p < eol; p++)
	{
	  other = *p;

	  if (other->val == TRUE)
	    continue;

	  assert (other->val == FALSE);

	  u = LIT2VAR (other);
	  if (u->mark)
	    continue;
	  
	  u->mark = 1;
	  inc_score (u);
	  use_var (u);

	  if (u->level == level)
	    {
	      open++;
	    }
	  else 
	    {
	      push_var_as_marked (u);

	      if (u->level)
		{
		  /* The statistics counter 'nonminimizedllits' sums up the
		   * number of literals that would be added if only the
		   * 'first UIP' scheme for learned clauses would be used
		   * and no clause minimization.
		   */
		  nonminimizedllits++;

		  if (u->level < minlevel)
		    minlevel = u->level;

		  siglevels |= hashlevel (u->level);
		}
	      else
		{
		  assert (!u->level);
		  assert (u->reason);
		}
	    }
	}

      do
	{
	  if (q == trail)
	    {
	      uip = 0;
	      goto DONE_FIRST_UIP;
	    }

	  this = *--q;
	  uip = LIT2VAR (this);
	}
      while (!uip->mark);

      uip->mark = 0;

      c = var2reason (uip);
#ifdef NO_BINARY_CLAUSES
      if (c == &impl)
	resetimpl ();
#endif
     open--;
     if ((!open && level) || !c)
	break;

     assert (c);
    }

DONE_FIRST_UIP:

  if (uip)
    {
      assert (level);
      this = VAR2LIT (uip);
      this += (this->val == TRUE);
      nonminimizedllits++;
      minimizedllits++;
      add_lit (this);
#ifdef STATS
      if (uip->reason)
	uips++;
#endif
    }
  else
    assert (!level);

  /* Second, try to mark more intermediate variables, with the goal to
   * minimize the conflict clause.  This is a DFS from already marked
   * variables backward through the implication graph.  It tries to reach
   * other marked variables.  If the search reaches an unmarked decision
   * variable or a variable assigned below the minimum level of variables in
   * the first uip learned clause or a level on which no variable has been
   * marked, then the variable from which the DFS is started is not
   * redundant.  Otherwise the start variable is redundant and will
   * eventually be removed from the learned clause in step 4.  We initially
   * implemented BFS, but then profiling revelead that this step is a bottle
   * neck for certain incremental applications.  After switching to DFS this
   * hot spot went away.
   */
  orig = mhead - marked;
  for (i = 0; i < orig; i++)
    {
      start = marked[i];

      assert (start->mark);
      assert (start != uip);
      assert (start->level < level);

      if (!start->reason)
	continue;

      old = mhead - marked;
      assert (dhead == dfs);
      push (start);

      while (dhead > dfs)
	{
	  u = pop ();
	  assert (u->mark);

	  c = var2reason (u);
#ifdef NO_BINARY_CLAUSES
	  if (c == &impl)
	    resetimpl ();
#endif
	  if (!c || 
	      ((l = u->level) && 
	       (l < minlevel || ((hashlevel (l) & ~siglevels)))))
	    {
	      while (mhead > marked + old)	/* reset all marked */
		(*--mhead)->mark = 0;

	      dhead = dfs;		/* and DFS stack */
	      break;
	    }

	  eol = end_of_lits (c);
	  for (p = c->lits; p < eol; p++)
	    {
	      v = LIT2VAR (*p);
	      if (v->mark)
		continue;

	      mark_var (v);
	      push (v);
	    }
	}
    }

  for (m = marked; m < mhead; m++)
    {
      v = *m;

      assert (v->mark);
      assert (!v->resolved);

      use_var (v);

      c = var2reason (v);
      if (!c)
	continue;

#ifdef NO_BINARY_CLAUSES
      if (c == &impl)
	resetimpl ();
#endif
      eol = end_of_lits (c);
      for (p = c->lits; p < eol; p++)
	{
	  other = *p;

	  u = LIT2VAR (other);
	  if (!u->level)
	    continue;

	  if (!u->mark)		/* 'MARKTEST' */
	    break;
	}

      if (p != eol)
	continue;

      add_antecedent (c);
      v->resolved = 1;
    }

  for (m = marked; m < mhead; m++)
    {
      v = *m;

      assert (v->mark);
      v->mark = 0;

      if (v->resolved)
	{
	  v->resolved = 0;
	  continue;
	}

      this = VAR2LIT (v);
      if (this->val == TRUE)
	this++;			/* actually NOTLIT */

      add_lit (this);
      minimizedllits++;
    }

  assert (ahead <= eoa);
  assert (rhead <= eor);

  mhead = marked;
}

static void
fanalyze (void)
{
  Lit ** eol, ** p, * lit;
  Cls * cls, * reason;
  Var * v, * u;
  int next;

  double start = picosat_time_stamp ();

  assert (failed_assumption);
  assert (failed_assumption->val == FALSE);

  v = LIT2VAR (failed_assumption);
  reason = var2reason (v);
  if (!reason) return;
#ifdef NO_BINARY_CLAUSES
  if (reason == &impl)
    resetimpl ();
#endif

  eol = end_of_lits (reason);
  for (p = reason->lits; p != eol; p++)
    {
      lit = *p;
      u = LIT2VAR (lit);
      if (u == v) continue;
      if (u->reason) break;
    }
  if (p == eol) return;

  assert (ahead == added);
  assert (mhead == marked);
  assert (rhead == resolved);

  next = 0;
  mark_var (v);
  add_lit (NOTLIT (failed_assumption));

  do
    {
      v = marked[next++];
      use_var (v);
      if (v->reason)
	{
	  reason = var2reason (v);
#ifdef NO_BINARY_CLAUSES
	  if (reason == &impl)
	    resetimpl ();
#endif
	  add_antecedent (reason);
	  eol = end_of_lits (reason);
	  for (p = reason->lits; p != eol; p++)
	    {
	      lit = *p;
	      u = LIT2VAR (lit);
	      if (u == v) continue;
	      if (u->mark) continue;
	      mark_var (u);
	    }
	}
      else
	{
	  lit = VAR2LIT (v);
	  if (lit->val == TRUE) lit = NOTLIT (lit);
	  add_lit (lit);
	}
    } 
  while (marked + next < mhead);

  cls = add_simplified_clause (1);
  v = LIT2VAR (failed_assumption);
  reason = v->reason;
#ifdef NO_BINARY_CLAUSES
  if (!ISLITREASON (reason))
#endif
    {
      assert (reason->locked);
      reason->locked = 0;
      if (reason->learned && reason->size > 2)
	{
	  assert (llocked > 0);
	  llocked--;
	}
    }

#ifdef NO_BINARY_CLAUSES
  if (cls == &impl)
    {
      cls = impl2reason (NOTLIT (failed_assumption));
    }
  else
#endif
    {
      assert (cls->learned);
      assert (!cls->locked);
      cls->locked = 1;
      if (cls->size > 2)
	{
	  llocked++;
	  assert (llocked > 0);
	}
    }

  v->reason = cls;

  while (mhead > marked)
    (*--mhead)->mark = 0;

  if (verbosity)
    fprintf (out, "%sfanalyze took %.1f seconds\n", 
	     prefix, picosat_time_stamp () - start);
}

/* Propagate assignment of 'this' to 'FALSE' by visiting all binary clauses in
 * which 'this' occurs.
 */
inline static void
prop2 (Lit * this)
{
#ifdef NO_BINARY_CLAUSES
  Lit ** l, ** start;
  Ltk * lstk;
#else
  Cls * cls, ** p;
  Cls * next;
#endif
  Lit * other;
  Val tmp;

  assert (this->val == FALSE);

#ifdef NO_BINARY_CLAUSES
  lstk = LIT2IMPLS (this);
  start = lstk->start;
  l = start + lstk->count;
  while (l != start)
    {
      /* The counter 'visits' is the number of clauses that are
       * visited during propagations of assignments.
       */
      visits++;
#ifdef STATS
      bvisits++;
#endif
      other = *--l;
      tmp = other->val;

      if (tmp == TRUE)
	{
#ifdef STATS
	  othertrue++;
	  othertrue2++;
	  if (LIT2VAR (other)->level < level)
	    othertrue2u++;
#endif
	  continue;
	}

      if (tmp != FALSE)
	{
	  assign_forced (other, LIT2REASON (NOTLIT(this)));
	  continue;
	}

      if (conflict == &cimpl)
	resetcimpl ();
      conflict = setcimpl (this, other);
    }
#else
  /* Traverse all binary clauses with 'this'.  Head/Tail pointers for binary
   * clauses do not have to be modified here.
   */
  p = LIT2IMPLS (this);
  for (cls = *p; cls; cls = next)
    {
      visits++;
#ifdef STATS
      bvisits++;
#endif
      assert (!cls->collect);
#ifdef TRACE
      assert (!cls->collected);
#endif
      assert (cls->size == 2);
      
      other = cls->lits[0];
      if (other == this)
	{
	  next = cls->next[0];
	  other = cls->lits[1];
	}
      else
	next = cls->next[1];

      tmp = other->val;

      if (tmp == TRUE)
	{
#ifdef STATS
	  othertrue++;
	  othertrue2++;
	  if (LIT2VAR (other)->level < level)
	    othertrue2u++;
#endif
	  continue;
	}

      if (tmp == FALSE)
	conflict = cls;
      else
	assign_forced (other, cls);	/* unit clause */
    }
#endif /* !defined(NO_BINARY_CLAUSES) */
}

#ifndef NDSC
static int
should_disconnect_head_tail (Lit * lit)
{
  unsigned lit_level;
  Var * v;

  assert (lit->val == TRUE);

  v = LIT2VAR (lit);
  lit_level = v->level;

  if (!lit_level)
    return 1;

#ifndef NFL
  if (simplifying)
    return 0;
#endif

  return lit_level < level;
}
#endif

inline static void
propl (Lit * this)
{
  Lit **l, *other, *prev, *new_lit, **eol;
  Cls *next, **htp_ptr, **new_htp_ptr;
  Cls *cls;
#ifdef STATS
  unsigned size;
#endif

  htp_ptr = LIT2HTPS (this);
  assert (this->val == FALSE);

  /* Traverse all non binary clauses with 'this'.  Head/Tail pointers are
   * updated as well.
   */
  for (cls = *htp_ptr; cls; cls = next)
    {
      visits++;
#ifdef STATS
      size = cls->size;
      assert (size >= 3);
      traversals++;	/* other is dereferenced at least */

      if (size == 3)
	tvisits++;
      else if (size >= 4)
	{
	  lvisits++;
	  ltraversals++;
	}
#endif
#ifdef TRACE
      assert (!cls->collected);
#endif
      assert (cls->size > 0);

      other = cls->lits[0];
      if (other != this)
	{
	  assert (cls->size != 1);
	  cls->lits[0] = this;
	  cls->lits[1] = other;
	  next = cls->next[1];
	  cls->next[1] = cls->next[0];
	  cls->next[0] = next;
	}
      else if (cls->size == 1)		/* With assumptions we need to
	                                 * traverse unit clauses as well.
					 */
	{
	  assert (!conflict);
	  conflict = cls;
	  break;
	}
      else
	{
	  assert (other == this && cls->size > 1);
	  other = cls->lits[1];
	  next = cls->next[0];
	}
      assert (other == cls->lits[1]);
      assert (this == cls->lits[0]);
      assert (next == cls->next[0]);
      assert (!cls->collect);

      if (other->val == TRUE)
	{
#ifdef STATS
	  othertrue++;
	  othertruel++;
#endif
#ifndef NDSC
	  if (should_disconnect_head_tail (other))
	    {
	      new_htp_ptr = LIT2DHTPS (other);
	      cls->next[0] = *new_htp_ptr;
	      *new_htp_ptr = cls;
#ifdef STATS
	      othertruelu++;
#endif
	      *htp_ptr = next;
	      continue;
	    }
#endif
	  htp_ptr = cls->next;
	  continue;
	}

      l = cls->lits + 1;
      eol = cls->lits + cls->size;
      prev = this;

      while (++l != eol)
	{
#ifdef STATS
	  if (size >= 3)
	    {
	      traversals++;
	      if (size > 3)
		ltraversals++;
	    }
#endif
	  new_lit = *l;
	  *l = prev;
	  prev = new_lit;
	  if (new_lit->val != FALSE) break;
	}

      if (l == eol)
	{
	  while (l > cls->lits + 2) 
	    {
	      new_lit = *--l;
	      *l = prev;
	      prev = new_lit;
	    }
	  assert (cls->lits[0] == this);

	  assert (other == cls->lits[1]);
	  if (other->val == FALSE)	/* found conflict */
	    {
	      assert (!conflict);
	      conflict = cls;
	      return;
	    }

	  assign_forced (other, cls);		/* unit clause */
	  htp_ptr = cls->next;
	}
      else
	{
	  assert (new_lit->val == TRUE || new_lit->val == UNDEF);
	  cls->lits[0] = new_lit;
	  // *l = this;
	  new_htp_ptr = LIT2HTPS (new_lit);
	  cls->next[0] = *new_htp_ptr;
	  *new_htp_ptr = cls;
	  *htp_ptr = next;
	}
    }
}

#ifndef NADC

static unsigned primes[] = { 996293, 330643, 753947, 500873 };

#define PRIMES ((sizeof primes)/sizeof *primes)

static unsigned
hash_ado (Lit ** ado, unsigned salt)
{
  unsigned i, res, tmp;
  Lit ** p, * lit;

  assert (salt < PRIMES);

  i = salt;
  res = 0;

  for (p = ado; (lit = *p); p++)
    {
      assert (lit->val);

      tmp = res >> 31;
      res <<= 1;

      if (lit->val > 0)
	res |= 1;

      assert (i < PRIMES);
      res *= primes[i++];
      if (i == PRIMES)
	i = 0;

      res += tmp;
    }

  return res & (szadotab - 1);
}

static unsigned
cmp_ado (Lit ** a, Lit ** b)
{
  Lit ** p, ** q, * l, * k;
  int res;

  for (p = a, q = b; (l = *p); p++, q++)
    {
      k = *q;
      assert (k);
      if ((res = (l->val - k->val)))
	return res;
    }

  assert (!*q);

  return 0;
}

static Lit ***
find_ado (Lit ** ado)
{
  Lit *** res, ** other;
  unsigned pos, delta;

  pos = hash_ado (ado, 0);
  assert (pos < szadotab);
  res = adotab + pos;

  other = *res;
  if (!other || !cmp_ado (other, ado))
    return res;

  delta = hash_ado (ado, 1);
  if (!(delta & 1))
    delta++;

  assert (delta & 1);
  assert (delta < szadotab);

  for (;;)
    {
      pos += delta;
      if (pos >= szadotab)
	pos -= szadotab;

      assert (pos < szadotab);
      res = adotab + pos;
      other = *res;
      if (!other || !cmp_ado (other, ado))
	return res;
    }
}

static void
enlarge_adotab (void)
{
  /* TODO make this generic */

  ABORTIF (szadotab, 
           "internal: all different objects table needs larger initial size");
  assert (!nadotab);
  szadotab = 10000;
  NEWN (adotab, szadotab);
  CLRN (adotab, szadotab);
}

static int
propado (Var * v)
{
  Lit ** p, ** q, *** adotabpos, **ado, * lit;
  Var * u;

  if (level && adodisabled)
    return 1;

  assert (!conflict);
  assert (!adoconflict);
  assert (VAR2LIT (v)->val != UNDEF);
  assert (!v->adotabpos);

  if (!v->ado)
    return 1;

  assert (v->inado);

  for (p = v->ado; (lit = *p); p++)
    if (lit->val == UNDEF)
      {
	u = LIT2VAR (lit);
	assert (!u->ado);
	u->ado = v->ado;
	v->ado = 0;

	return 1;
      }

  if (4 * nadotab >= 3 * szadotab)	/* at least 75% filled */
    enlarge_adotab ();

  adotabpos = find_ado (v->ado);
  ado = *adotabpos;

  if (!ado)
    {
      nadotab++;
      v->adotabpos = adotabpos;
      *adotabpos = v->ado;
      return 1;
    }

  assert (ado != v->ado);

  adoconflict = new_clause (2 * llength (ado), 1);
  q = adoconflict->lits;

  for (p = ado; (lit = *p); p++)
    *q++ = lit->val == FALSE ? lit : NOTLIT (lit);

  for (p = v->ado; (lit = *p); p++)
    *q++ = lit->val == FALSE ? lit : NOTLIT (lit);

  assert (q == ENDOFCLS (adoconflict));
  conflict = adoconflict;
  adoconflicts++;
  return 0;
}

#endif

static void
bcp (void)
{
  int props = 0;
  assert (!conflict);

  if (mtcls)
    return;

  for (;;)
    {
      if (ttail2 < thead)	/* prioritize implications */
	{
	  props++;
	  prop2 (NOTLIT (*ttail2++));
	}
      else if (ttail < thead)	/* unit clauses or clauses with length > 2 */
	{
	  if (conflict) break;
	  propl (NOTLIT (*ttail++));
	  if (conflict) break;
	}
#ifndef NADC
      else if (ttailado < thead)
	{
	  if (conflict) break;
	  propado (LIT2VAR (*ttailado++));
	  if (conflict) break;
	}
#endif
      else
	break;		/* all assignments propagated, so break */
    }

  propagations += props;
}

static unsigned
drive (void)
{
  unsigned res, vlevel;
  Lit **p;
  Var *v;

  res = 0;
  for (p = added; p < ahead; p++)
    {
      v = LIT2VAR (*p);
      vlevel = v->level;
      assert (vlevel <= level);
      if (vlevel < level && vlevel > res)
	res = vlevel;
    }

  return res;
}

#ifdef VISCORES

static void
viscores (void)
{
  Rnk *p, *eor = rnks + max_var;
  char name[100], cmd[200];
  FILE * data;
  Flt s;
  int i;

  for (p = rnks + 1; p <= eor; p++)
    {
      s = p->score;
      if (s == INFFLT)
	continue;
      s = mulflt (s, nvinc);
      assert (flt2double (s) <= 1.0);
    }

  sprintf (name, "/tmp/picosat-viscores/data/%08u", conflicts);
  sprintf (cmd, "sort -n|nl>%s", name);

  data = popen (cmd, "w");
  for (p = rnks + 1; p <= eor; p++)
    {
      s = p->score;
      if (s == INFFLT)
	continue;
      s = mulflt (s, nvinc);
      fprintf (data, "%lf %d\n", 100.0 * flt2double (s), (int)(p - rnks));
    }
  fflush (data);
  pclose (data);

  for (i = 0; i < 8; i++)
    {
      sprintf (cmd, "awk '$3%%8==%d' %s>%s.%d", i, name, name, i);
      system (cmd);
    }

  fprintf (fviscores, "set title \"%u\"\n", conflicts);
  fprintf (fviscores, "plot [0:%u] 0, 100 * (1 - 1/1.1), 100", max_var);

  for (i = 0; i < 8; i++)
    fprintf (fviscores, 
             ", \"%s.%d\" using 1:2:3 with labels tc lt %d", 
	     name, i, i + 1);

  fputc ('\n', fviscores);
  fflush (fviscores);
#ifndef WRITEGIF
  usleep (50000);		/* refresh rate of 20 Hz */
#endif
}

#endif

static void
crescore (void)
{
  Cls **p, *cls;
  Act *a;
  Flt factor;
  int l = log2flt (cinc);
  assert (l > 0);
  factor = base2flt (1, -l);

  for (p = lclauses; p != lhead; p++)
    {
      cls = *p;

      if (!cls)
	continue;

#ifdef TRACE
      if (cls->collected)
	continue;
#endif
      assert (cls->learned);

      if (cls->size <= 2)
	continue;

      a = CLS2ACT (cls);
      *a = mulflt (*a, factor);
    }

  cinc = mulflt (cinc, factor);
}

static void
inc_vinc (void)
{
#ifdef VISCORES
  nvinc = mulflt (nvinc, fvinc);
#endif
  vinc = mulflt (vinc, ifvinc);
}

inline static void
inc_max_var (void)
{
  Lit *lit;
  Rnk *r;
  Var *v;

  assert (max_var < size_vars);

  max_var++;			/* new index of variable */
  assert (max_var);		/* no unsigned overflow */

  if (max_var == size_vars)
    enlarge (size_vars + (size_vars + 3) / 4);	/* increase by 25% */

  assert (max_var < size_vars);

  lit = lits + 2 * max_var;
  lit[0].val = lit[1].val = UNDEF;

  memset (htps + 2 * max_var, 0, 2 * sizeof *htps);
#ifndef NDSC
  memset (dhtps + 2 * max_var, 0, 2 * sizeof *dhtps);
#endif
  memset (impls + 2 * max_var, 0, 2 * sizeof *impls);
  memset (jwh + 2 * max_var, 0, 2 * sizeof *jwh);

  v = vars + max_var;		/* initialize variable components */
  CLR (v);

  r = rnks + max_var;		/* initialize rank */
  CLR (r);

  hpush (r);
}

static void
force (Cls * cls)
{
  Lit ** p, ** eol, * lit, * forced;
  Cls * reason;

  forced = 0;
  reason = cls;

  eol = end_of_lits (cls);
  for (p = cls->lits; p < eol; p++)
    {
      lit = *p;
      if (lit->val == UNDEF)
	{
	  assert (!forced);
	  forced = lit;
#ifdef NO_BINARY_CLAUSES
	  if (cls == &impl)
	    reason = LIT2REASON (NOTLIT (p[p == cls->lits ? 1 : -1]));
#endif
	}
      else
	assert (lit->val == FALSE);
    }

#ifdef NO_BINARY_CLAUSES
  if (cls == &impl)
    resetimpl ();
#endif
  if (!forced)
    return;

  assign_forced (forced, reason);
}

static void
inc_lreduce (void)
{
#ifdef STATS
  inclreduces++;
#endif
  lreduce *= FREDUCE;
  lreduce /= 100;
  report (1, '+');
}

static void
backtrack (void)
{
  unsigned new_level;
  Cls * cls;

  conflicts++;
  LOG (fprintf (out, "%sconflict ", prefix); dumpclsnl (conflict));

  analyze ();
  new_level = drive ();
  // TODO: why not? assert (new_level != 1  || (ahead - added) == 2);
  cls = add_simplified_clause (1);
  undo (new_level);
  force (cls);

  if (
#ifndef NFL
    !simplifying && 
#endif
    !--lreduceadjustcnt)
    {
      /* With FREDUCE==110 and FREDADJ=121 we stir 'lreduce' to be
       * proportional to 'sqrt(conflicts)'.  In earlier version we actually
       * used  'FREDADJ=150', which results in 'lreduce' to approximate
       * 'conflicts^(log(1.1)/log(1.5))' which is close to the fourth root
       * of 'conflicts', since log(1.1)/log(1.5)=0.235 (as observed by
       * Donald Knuth). The square root is the same we get by a Glucose
       * style increase, which simply adds a constant at every reduction.
       * This would be way simpler to implement but for now we keep the more
       * complicated code using the adjust increments and counters.
       */
      lreduceadjustinc *= FREDADJ; lreduceadjustinc /= 100; lreduceadjustcnt
      = lreduceadjustinc;
      inc_lreduce ();
    }

  if (verbosity >= 4 && !(conflicts % 1000))
    report (4, 'C');
}

static void
inc_cinc (void)
{
  cinc = mulflt (cinc, fcinc);
  if (lcinc < cinc)
    crescore ();
}

static void
incincs (void)
{
  inc_vinc ();
  inc_cinc ();
#ifdef VISCORES
  viscores ();
#endif
}

static void
disconnect_clause (Cls * cls)
{
  assert (cls->connected);

  if (cls->size > 2)
    {
      if (cls->learned)
	{
	  assert (nlclauses > 0);
	  nlclauses--;

	  assert (llits >= cls->size);
	  llits -= cls->size;
	}
      else
	{
	  assert (noclauses > 0);
	  noclauses--;

	  assert (olits >= cls->size);
	  olits -= cls->size;
	}
    }

#ifndef NDEBUG
  cls->connected = 0;
#endif
}

static int
clause_is_toplevel_satisfied (Cls * cls)
{
  Lit *lit, **p, **eol = end_of_lits (cls);
  Var *v;

  for (p = cls->lits; p < eol; p++)
    {
      lit = *p;
      if (lit->val == TRUE)
	{
	  v = LIT2VAR (lit);
	  if (!v->level)
	    return 1;
	}
    }

  return 0;
}

static int
collect_clause (Cls * cls)
{
  assert (cls->collect);
  cls->collect = 0;

#ifdef TRACE
  assert (!cls->collected);
  cls->collected = 1;
#endif
  disconnect_clause (cls);

#ifdef TRACE
  if (trace && (!cls->learned || cls->used))
    return 0;
#endif
  delete_clause (cls);

  return 1;
}

static size_t
collect_clauses (void)
{
  Cls *cls, **p, **q, * next;
  Lit * lit, * eol;
  size_t res;
  int i;

  res = current_bytes;

  eol = lits + 2 * max_var + 1;
  for (lit = lits + 2; lit <= eol; lit++)
    {
      for (i = 0; i <= 1; i++)
	{
	  if (i)
	    {
#ifdef NO_BINARY_CLAUSES
	      Ltk * lstk = LIT2IMPLS (lit);
	      Lit ** r, ** s;
	      r = lstk->start;
	      if (lit->val != TRUE || LIT2VAR (lit)->level)
		for (s = r; s < lstk->start + lstk->count; s++)
		  {
		    Lit * other = *s;
		    Var *v = LIT2VAR (other);
		    if (v->level || other->val != TRUE)
		      *r++ = other;
		  }
	      lstk->count = r - lstk->start;
	      continue;
#else
	      p = LIT2IMPLS (lit);
#endif
	    }
	  else
	    p = LIT2HTPS (lit);

	  for (cls = *p; cls; cls = next)
	    {
	      q = cls->next;
	      if (cls->lits[0] != lit)
		q++;

	      next = *q;
	      if (cls->collect)
		*p = next;
	      else
		p = q;
	    }
	}
    }

#ifndef NDSC
  for (lit = lits + 2; lit <= eol; lit++)
    {
      p = LIT2DHTPS (lit); 
      while ((cls = *p))
	{
	  Lit * other = cls->lits[0];
	  if (other == lit)
	    {
	      q = cls->next + 1;
	    }
	  else
	    {
	      assert (cls->lits[1] == lit);
	      q = cls->next;
	    }

	  if (cls->collect)
	    *p = *q;
	  else
	    p = q;
	}
    }
#endif

  for (p = SOC; p != EOC; p = NXC (p))
    {
      cls = *p;

      if (!cls)
	continue;

      if (!cls->collect)
	continue;

      if (collect_clause (cls))
	*p = 0;
    }

#ifdef TRACE
  if (!trace)
#endif
    {
      q = oclauses;
      for (p = q; p < ohead; p++)
	if ((cls = *p))
	  *q++ = cls;
      ohead = q;

      q = lclauses;
      for (p = q; p < lhead; p++)
	if ((cls = *p))
	  *q++ = cls;
      lhead = q;
    }

  assert (current_bytes <= res);
  res -= current_bytes;
  recycled += res;

  LOG (fprintf (out, "%scollected %ld bytes\n", prefix, (long)res));

  return res;
}

static int
need_to_reduce (void)
{
  return nlclauses >= reduce_limit_on_lclauses ();
}

#ifdef NLUBY

static void
inc_drestart (void)
{
  drestart *= FRESTART;
  drestart /= 100;

  if (drestart >= MAXRESTART)
    drestart = MAXRESTART;
}

static void
inc_ddrestart (void)
{
  ddrestart *= FRESTART;
  ddrestart /= 100;

  if (ddrestart >= MAXRESTART)
    ddrestart = MAXRESTART;
}

#else

static int
luby (int i)
{
  int k;
  for (k = 1; k < 32; k++)
    if (i == (1 << k) - 1)
      return 1 << (k - 1);

  for (k = 1;; k++)
    if ((1 << (k - 1)) <= i && i < (1 << k) - 1)
      return luby (i - (1 << (k-1)) + 1);
}

#endif

#ifndef NLUBY
static void
inc_lrestart (int skip)
{
  unsigned delta;

  delta = 100 * luby (++lubycnt);
  lrestart = conflicts + delta;

  if (waslubymaxdelta)
    report (1, skip ? 'N' : 'R');
  else
    report (2, skip ? 'n' : 'r');

  if (delta > lubymaxdelta)
    {
      lubymaxdelta = delta;
      waslubymaxdelta = 1;
    }
  else
    waslubymaxdelta = 0;
}
#endif

static void
init_restart (void)
{
#ifdef NLUBY
  /* TODO: why is it better in incremental usage to have smaller initial
   * outer restart interval?
   */
  ddrestart = calls > 1 ? MINRESTART : 1000;
  drestart = MINRESTART;
  lrestart = conflicts + drestart;
#else
  lubycnt = 0;
  lubymaxdelta = 0;
  waslubymaxdelta = 0;
  inc_lrestart (0);
#endif
}

static void
restart (void)
{
  int skip; 
#ifdef NLUBY
  char kind;
  int outer;
 
  inc_drestart ();
  outer = (drestart >= ddrestart);

  if (outer)
    skip = very_high_agility ();
  else
    skip = high_agility ();
#else
  skip = medium_agility ();
#endif

#ifdef STATS
  if (skip)
    skippedrestarts++;
#endif

  assert (conflicts >= lrestart);

  if (!skip)
    {
      restarts++;
      assert (level > 1);
      LOG (fprintf (out, "%srestart %u\n", prefix, restarts));
      undo (0);
    }

#ifdef NLUBY
  if (outer)
    {
      kind = skip ? 'N' : 'R';
      inc_ddrestart ();
      drestart = MINRESTART;
    }
  else  if (skip)
    {
      kind = 'n';
    }
  else
    {
      kind = 'r';
    }

  assert (drestart <= MAXRESTART);
  lrestart = conflicts + drestart;
  assert (lrestart > conflicts);

  report (outer ? 1 : 2, kind);
#else
  inc_lrestart (skip);
#endif
}

inline static void
assign_decision (Lit * lit)
{
  assert (!conflict);

  level++;

  LOG (fprintf (out, "%snew level %u\n", prefix, level));
  LOG (fprintf (out,
		"%sassign %d at level %d <= DECISION\n",
		prefix, LIT2INT (lit), level));

  assign (lit, 0);
}

#ifndef NFL

static int
lit_has_binary_clauses (Lit * lit)
{
#ifdef NO_BINARY_CLAUSES
  Ltk* lstk = LIT2IMPLS (lit);
  return lstk->count != 0;
#else
  return *LIT2IMPLS (lit) != 0;
#endif
}

static void
flbcp (void)
{
#ifdef STATS
  unsigned long long propagaions_before_bcp = propagations;
#endif
  bcp ();
#ifdef STATS
  flprops += propagations - propagaions_before_bcp;
#endif
}

inline static int
cmp_inverse_rnk (Rnk * a, Rnk * b)
{
  return -cmp_rnk (a, b);
}

inline static Flt
rnk2jwh (Rnk * r)
{
  Flt res, sum, pjwh, njwh;
  Lit * plit, * nlit;

  plit = RNK2LIT (r);
  nlit = plit + 1;
  
  pjwh = *LIT2JWH (plit);
  njwh = *LIT2JWH (nlit);

  res = mulflt (pjwh, njwh);

  sum = addflt (pjwh, njwh);
  sum = mulflt (sum, base2flt (1, -10));
  res = addflt (res, sum);

  return res;
}

static int
cmp_inverse_jwh_rnk (Rnk * r, Rnk * s)
{
  Flt a = rnk2jwh (r);
  Flt b = rnk2jwh (s);
  int res = cmpflt (a, b);

  if (res)
    return -res;

  return cmp_inverse_rnk (r, s);
}

static void
faillits (void)
{
  unsigned i, j, old_trail_count, common, saved_count;
  unsigned new_saved_size, oldladded = ladded;
  unsigned long long limit, delta;
  Lit * lit, * other, * pivot;
  Rnk * r, ** p, ** q;
  int new_trail_count;
  double started;

  if (heap + 1 >= hhead)
    return;

  if (propagations < fllimit)
    return;

  sflush ();
  started = seconds;

  flcalls++;
#ifdef STATSA
  flrounds++;
#endif
  delta = propagations/10;
  if (delta >= 100*1000*1000) delta = 100*1000*1000;
  else if (delta <= 100*1000) delta = 100*1000;

  limit = propagations + delta;
  fllimit = propagations;

  assert (!level);
  assert (simplifying);

  if (flcalls <= 1)
    SORT (Rnk *, cmp_inverse_jwh_rnk, heap + 1, hhead - (heap + 1));
  else
    SORT (Rnk *, cmp_inverse_rnk, heap + 1, hhead - (heap + 1));

  i = 1;		/* NOTE: heap starts at position '1' */

  while (propagations < limit)
    {
      if (heap + i == hhead)
	{
	  if (ladded == oldladded)
	    break;

	  i = 1;
#ifdef STATS
	  flrounds++;
#endif
	  oldladded = ladded;
	}

      assert (heap + i < hhead);

      r = heap[i++];
      lit = RNK2LIT (r);

      if (lit->val)
	continue;

      if (!lit_has_binary_clauses (NOTLIT (lit)))
	{
#ifdef STATS
	  flskipped++;
#endif
	  continue;
	}

#ifdef STATS
      fltried++;
#endif
      LOG (fprintf (out, "%strying %d as failed literal\n",
	    prefix, LIT2INT (lit)));

      assign_decision (lit);
      old_trail_count = thead - trail;
      flbcp ();

      if (conflict)
	{
EXPLICITLY_FAILED_LITERAL:
	  LOG (fprintf (out, "%sfound explicitly failed literal %d\n",
		prefix, LIT2INT (lit)));

	  failedlits++;
	  efailedlits++;

	  backtrack ();
	  flbcp ();

	  if (!conflict)
	    continue;

CONTRADICTION:
	  assert (!level);
	  backtrack ();
	  assert (mtcls);

	  goto RETURN;
	}

      if (propagations >= limit)
	{
	  undo (0);
	  break;
	}

      lit = NOTLIT (lit);

      if (!lit_has_binary_clauses (NOTLIT (lit)))
	{
#ifdef STATS
	  flskipped++;
#endif
	  undo (0);
	  continue;
	}

#ifdef STATS
      fltried++;
#endif
      LOG (fprintf (out, "%strying %d as failed literals\n",
	    prefix, LIT2INT (lit)));

      new_trail_count = thead - trail;
      saved_count = new_trail_count - old_trail_count;

      if (saved_count > saved_size)
	{
	  new_saved_size = saved_size ? 2 * saved_size : 1;
	  while (saved_count > new_saved_size)
	    new_saved_size *= 2;

	  RESIZEN (saved, saved_size, new_saved_size);
	  saved_size = new_saved_size;
	}

      for (j = 0; j < saved_count; j++)
	saved[j] = trail[old_trail_count + j];

      undo (0);

      assign_decision (lit);
      flbcp ();

      if (conflict)
	goto EXPLICITLY_FAILED_LITERAL;

      pivot = (thead - trail <= new_trail_count) ? lit : NOTLIT (lit);

      common = 0;
      for (j = 0; j < saved_count; j++)
	if ((other = saved[j])->val == TRUE)
	  saved[common++] = other;

      undo (0);

      LOG (if (common)
	    fprintf (out, 
		      "%sfound %d literals implied by %d and %d\n",
		      prefix, common, 
		      LIT2INT (NOTLIT (lit)), LIT2INT (lit)));

#if 1 // set to zero to disable 'lifting'
      for (j = 0; 
	   j < common 
	  /* TODO: For some Velev benchmarks, extracting the common implicit
	   * failed literals took quite some time.  This needs to be fixed by
	   * a dedicated analyzer.  Up to then we bound the number of
	   * propagations in this loop as well.
	   */
	   && propagations < limit + delta
	   ; j++)
	{
	  other = saved[j];

	  if (other->val == TRUE)
	    continue;

	  assert (!other->val);

	  LOG (fprintf (out, 
			"%sforcing %d as forced implicitly failed literal\n",
			prefix, LIT2INT (other)));

	  assert (pivot != NOTLIT (other));
	  assert (pivot != other);

	  assign_decision (NOTLIT (other));
	  flbcp ();

	  assert (level == 1);

	  if (conflict)
	    {
	      backtrack ();
	      assert (!level);
	    }
	  else
	    {
	      assign_decision (pivot);
	      flbcp ();

	      backtrack ();

	      if (level)
		{
		  assert (level == 1);

		  flbcp ();

		  if (conflict)
		    {
		      backtrack ();
		      assert (!level);
		    }
		  else
		    {
		      assign_decision (NOTLIT (pivot));
		      flbcp ();
		      backtrack ();

		      if (level)
			{
			  assert (level == 1);
			  flbcp ();

			  if (!conflict)
			    {
#ifdef STATS
			      floopsed++;
#endif
			      undo (0);
			      continue;
			    }

			  backtrack ();
			}

		      assert (!level);
		    }

		  assert (!level);
		}
	    }
	  assert (!level);
	  flbcp ();

	  failedlits++;
	  ifailedlits++;

	  if (conflict)
	    goto CONTRADICTION;
	}
#endif
    }

  fllimit += 9 * (propagations - fllimit);	/* 10% for failed literals */

RETURN:

  /* First flush top level assigned literals.  Those are prohibited from
   * being pushed up the heap during 'faillits' since 'simplifying' is set.
   */
  assert (heap < hhead);
  for (p = q = heap + 1; p < hhead; p++)
    {
      r = *p;
      lit = RNK2LIT (r);
      if (lit->val)
       	r->pos = 0;
      else
	*q++ = r;
    }

  /* Then resort with respect to EVSIDS score and fix positions.
   */
  SORT (Rnk *, cmp_inverse_rnk, heap + 1, hhead - (heap + 1));
  for (p = heap + 1; p < hhead; p++)
    (*p)->pos = p - heap;

  sflush ();
  flseconds += seconds - started;
}

#endif

static void
simplify (int forced)
{
  Lit * lit, * notlit, ** t;
  unsigned collect, delta;
#ifdef STATS
  size_t bytes_collected;
#endif
  int * q, ilit;
  Cls **p, *c;
  Var * v;

#ifndef NDEDBUG
  (void) forced;
#endif

  assert (!mtcls);
  assert (!satisfied ());
  assert (forced || lsimplify <= propagations);
  assert (forced || fsimplify <= fixed);

  if (level)
    undo (0);
#ifndef NFL
  simplifying = 1;
  faillits ();
  simplifying = 0;

  if (mtcls)
    return;
#endif

  if (cils != cilshead)
    {
      assert (ttail == thead);
      assert (ttail2 == thead);
      ttail = trail;
      for (t = trail; t < thead; t++)
	{
	  lit = *t;
	  v = LIT2VAR (lit);
	  if (v->internal)
	    {
	      assert (LIT2INT (lit) < 0);
	      assert (lit->val == TRUE);
	      unassign (lit);
	    }
	  else
	    *ttail++ = lit;
	}
      ttail2 = thead = ttail;

      for (q = cils; q != cilshead; q++)
	{
	  ilit = *q;
	  assert (0 < ilit && ilit <= (int) max_var);
	  v = vars + ilit;
	  assert (v->internal);
	  v->level = 0;
	  v->reason = 0;
	  lit = int2lit (-ilit);
	  assert (lit->val == UNDEF);
	  lit->val = TRUE;
	  notlit = NOTLIT (lit);
	  assert (notlit->val == UNDEF);
	  notlit->val = FALSE;
	}
    }

  collect = 0;
  for (p = SOC; p != EOC; p = NXC (p))
    {
      c = *p;
      if (!c)
	continue;

#ifdef TRACE
      if (c->collected)
	continue;
#endif

      if (c->locked)
	continue;
      
      assert (!c->collect);
      if (clause_is_toplevel_satisfied (c))
	{
	  mark_clause_to_be_collected (c);
	  collect++;
	}
    }

  LOG (fprintf (out, "%scollecting %d clauses\n", prefix, collect));
#ifdef STATS
  bytes_collected = 
#endif
  collect_clauses ();
#ifdef STATS
  srecycled += bytes_collected;
#endif

  if (cils != cilshead)
    {
      for (q = cils; q != cilshead; q++)
	{
	  ilit = *q;
	  assert (0 < ilit && ilit <= (int) max_var);
	  assert (vars[ilit].internal);
	  if (rilshead == eorils)
	    ENLARGE (rils, rilshead, eorils);
	  *rilshead++ = ilit;
	  lit = int2lit (-ilit);
	  assert (lit->val == TRUE);
	  lit->val = UNDEF;
	  notlit = NOTLIT (lit);
	  assert (notlit->val == FALSE);
	  notlit->val = UNDEF;
	}
      cilshead = cils;
    }

  delta = 10 * (olits + llits) + 100000;
  if (delta > 2000000)
    delta = 2000000;
  lsimplify = propagations + delta;
  fsimplify = fixed;
  simps++;

  report (1, 's');
}

static void
iteration (void)
{
  assert (!level);
  assert (bcp_queue_is_empty ());
  assert (isimplify < fixed);

  iterations++;
  report (2, 'i');
#ifdef NLUBY
  drestart = MINRESTART;
  lrestart = conflicts + drestart;
#else
  init_restart ();
#endif
  isimplify = fixed;
}

static int
cmp_activity (Cls * c, Cls * d)
{
  Act a;
  Act b;

  assert (c->learned);
  assert (d->learned);

  a = *CLS2ACT (c);
  b = *CLS2ACT (d);

  if (a < b)
    return -1;

  if (b < a)
    return 1;

  /* Prefer shorter clauses.
   */
  if (c->size < d->size)
    return 1;

  if (c->size > d->size)
    return -1;

  return 0;
}

static void
reduce (unsigned percentage)
{
  unsigned rcount, lcollect, collect, target;
#ifdef STATS
  size_t bytes_collected;
#endif
  Cls **p, *cls;

  assert (rhead == resolved);

  lastreduceconflicts = conflicts;

  assert (percentage <= 100);
  LOG (fprintf (out, 
                "%sreducing %u%% learned clauses\n",
		prefix, percentage));

  while (nlclauses - llocked > (unsigned)(eor - resolved))
    ENLARGE (resolved, rhead, eor);

  collect = 0;
  lcollect = 0;

  for (p = ((fsimplify < fixed) ? SOC : lclauses); p != EOC; p = NXC (p))
    {
      cls = *p;
      if (!cls)
	continue;

#ifdef TRACE
      if (cls->collected)
	continue;
#endif

      if (cls->locked)
	continue;

      assert (!cls->collect);
      if (fsimplify < fixed && clause_is_toplevel_satisfied (cls))
	{
	  mark_clause_to_be_collected (cls);
	  collect++;

	  if (cls->learned && cls->size > 2)
	    lcollect++;

	  continue;
	}

      if (!cls->learned)
	continue;

      if (cls->size <= 2)
	continue;

      assert (rhead < eor);
      *rhead++ = cls;
    }
  assert (rhead <= eor);

  fsimplify = fixed;

  rcount = rhead - resolved;
  SORT (Cls *, cmp_activity, resolved, rcount);

  assert (nlclauses >= lcollect);
  target = nlclauses - lcollect + 1;

  target = (percentage * target + 99) / 100;

  if (target >= rcount)
    {
      target = rcount;
    }
  else
    {
      while (target > 0 && 
	     !cmp_activity (resolved[target - 1], resolved[target]))
	target--;
    }

  rhead = resolved + target;
  while (rhead > resolved)
    {
      cls = *--rhead;
      mark_clause_to_be_collected (cls);

      collect++;
      if (cls->learned && cls->size > 2)	/* just for consistency */
	lcollect++;
    }

  if (collect)
    {
      reductions++;
#ifdef STATS
      bytes_collected = 
#endif
      collect_clauses ();
#ifdef STATS
      rrecycled += bytes_collected;
#endif
      report (2, '-');
    }

  if (!lcollect)
    inc_lreduce ();		/* avoid dead lock */

  assert (rhead == resolved);
}

static void
init_reduce (void)
{
  // lreduce = loadded / 2;
  lreduce = 1000;

  if (lreduce < 100)
    lreduce = 100;

  if (verbosity)
    fprintf (out, 
             "%s\n%sinitial reduction limit %u clauses\n%s\n",
	     prefix, prefix, lreduce, prefix);
}

static unsigned
rng (void)
{
  unsigned res = srng;
  srng *= 1664525u;
  srng += 1013904223u;
  NOLOG (fprintf (out, "%srng () = %u\n", prefix, res));
  return res;
}

static unsigned
rrng (unsigned low, unsigned high)
{
  unsigned long long tmp;
  unsigned res, elements;
  assert (low <= high);
  elements = high - low + 1;
  tmp = rng ();
  tmp *= elements;
  tmp >>= 32;
  tmp += low;
  res = tmp;
  NOLOG (fprintf (out, "%srrng (%u, %u) = %u\n", prefix, low, high, res));
  assert (low <= res);
  assert (res <= high);
  return res;
}

static Lit *
decide_phase (Lit * lit)
{
  Lit * not_lit = NOTLIT (lit);
  Var *v = LIT2VAR (lit);

  assert (LIT2SGN (lit) > 0);
  if (v->usedefphase)
    {
      if (v->defphase)
	{
	  /* assign to TRUE */
	}
      else
	{
	  /* assign to FALSE */
	  lit = not_lit;
	}
    }
  else if (!v->assigned)
    {
#ifdef STATS
      staticphasedecisions++;
#endif
      if (defaultphase == POSPHASE)
	{
	  /* assign to TRUE */
	}
      else if (defaultphase == NEGPHASE)
	{
	  /* assign to FALSE */
	  lit = not_lit;
	}
      else if (defaultphase == RNDPHASE)
	{
	  /* randomly assign default phase */
	  if (rrng (1, 2) != 2)
	    lit = not_lit;
	}
      else if (*LIT2JWH(lit) <= *LIT2JWH (not_lit))
	{
	  /* assign to FALSE (Jeroslow-Wang says there are more short
	   * clauses with negative occurence of this variable, so satisfy
	   * those, to minimize BCP) 
	   */
	  lit = not_lit;
	}
      else
	{
	  /* assign to TRUE (... but strictly more positive occurrences) */
	}
    }
  else 
    {
      /* repeat last phase: phase saving heuristic */

      if (v->phase)
	{
	  /* assign to TRUE (last phase was TRUE as well) */
	}
      else
	{
	  /* assign to FALSE (last phase was FALSE as well) */
	  lit = not_lit;
	}
    }

  return lit;
}

static unsigned
gcd (unsigned a, unsigned b)
{
  unsigned tmp;

  assert (a);
  assert (b);

  if (a < b)
    {
      tmp = a;
      a = b;
      b = tmp;
    }

  while (b)
    {
      assert (a >= b);
      tmp = b;
      b = a % b;
      a = tmp;
    }

  return a;
}

static Lit *
rdecide (void)
{
  unsigned idx, delta, spread;
  Lit * res;

  spread = RDECIDE;
  if (rrng (1, spread) != 2)
    return 0;

  assert (1 <= max_var);
  idx = rrng (1, max_var);
  res = int2lit (idx);

  if (res->val != UNDEF)
    {
      delta = rrng (1, max_var);
      while (gcd (delta, max_var) != 1)
	delta--;

      assert (1 <= delta);
      assert (delta <= max_var);

      do {
	idx += delta;
	if (idx > max_var)
	  idx -= max_var;
	res = int2lit (idx);
      } while (res->val != UNDEF);
    }

#ifdef STATS
  rdecisions++;
#endif
  res = decide_phase (res);
  LOG (fprintf (out, "%srdecide %d\n", prefix, LIT2INT (res)));

  return res;
}

static Lit *
sdecide (void)
{
  Lit *res;
  Rnk *r;

  for (;;)
    {
      r = htop ();
      res = RNK2LIT (r);
      if (res->val == UNDEF) break;
      (void) hpop ();
      NOLOG (fprintf (out, 
                      "%shpop %u %u %u\n",
		      prefix, r - rnks,
		      FLTMANTISSA(r->score),
		      FLTEXPONENT(r->score)));
    }

#ifdef STATS
  sdecisions++;
#endif
  res = decide_phase (res);

  LOG (fprintf (out, "%ssdecide %d\n", prefix, LIT2INT (res)));

  return res;
}

static Lit *
adecide (void)
{
  Lit *lit;
  Var * v;

  assert (als < alshead);
  assert (!failed_assumption);

  while (alstail < alshead)
    {
      lit = *alstail++;

      if (lit->val == FALSE)
	{
	  failed_assumption = lit;
	  v = LIT2VAR (lit);

	  use_var (v);

	  LOG (fprintf (out, "%sfirst failed assumption %d\n",
			prefix, LIT2INT (failed_assumption)));
	  fanalyze ();
	  return 0;
	}

      if (lit->val == TRUE)
	{
	  v = LIT2VAR (lit);
	  if (v->level > adecidelevel)
	    adecidelevel = v->level;
	  continue;
	}

#ifdef STATS
      assumptions++;
#endif
      LOG (fprintf (out, "%sadecide %d\n", prefix, LIT2INT (lit)));
      adecidelevel = level + 1;

      return lit;
    }

  return 0;
}

static void
decide (void)
{
  Lit * lit;

  assert (!satisfied ());
  assert (!conflict);

  if (alstail < alshead && (lit = adecide ()))
    ;
  else if (failed_assumption)
    return;
  else if (satisfied ())
    return;
  else if (!(lit = rdecide ()))
    lit = sdecide ();

  assert (lit);
  assign_decision (lit);

  levelsum += level;
  decisions++;
}

static int
sat (int l)
{
  int count = 0, backtracked;

  if (!conflict)
    bcp ();

  if (conflict)
    backtrack ();

  if (mtcls)
    return PICOSAT_UNSATISFIABLE;

  if (satisfied ())
    goto SATISFIED;

  if (lsimplify <= propagations)
    simplify (0);

  if (mtcls)
    return PICOSAT_UNSATISFIABLE;

  if (satisfied ())
    goto SATISFIED;

  init_restart ();

  if (!lreduce)
    init_reduce ();

  isimplify = fixed;
  backtracked = 0;

  for (;;)
    {
      if (!conflict)
	bcp ();

      if (conflict)
	{
	  incincs ();
	  backtrack ();

	  if (mtcls)
	    return PICOSAT_UNSATISFIABLE;
	  backtracked = 1;
	  continue;
	}

      if (satisfied ())
	{
SATISFIED:
#ifndef NDEBUG
	  original_clauses_satisfied ();
	  assumptions_satisfied ();
#endif
	  return PICOSAT_SATISFIABLE;
	}

      if (backtracked)
	{
	  backtracked = 0;
	  if (!level && isimplify < fixed)
	    iteration ();
	}

      if (l >= 0 && count >= l)		/* decision limit reached ? */
	return PICOSAT_UNKNOWN;

      if (propagations >= lpropagations)/* propagation limit reached ? */
	return PICOSAT_UNKNOWN;

#ifndef NADC
      if (!adodisabled && adoconflicts >= adoconflictlimit)
	{
	  assert (bcp_queue_is_empty ());
	  return PICOSAT_UNKNOWN;
	}
#endif

      if (fsimplify < fixed && lsimplify <= propagations)
	{
	  simplify (0);
	  if (!bcp_queue_is_empty ())
	    continue;
#ifndef NFL
	  if (mtcls)
	    return PICOSAT_UNSATISFIABLE;

	  if (satisfied ())
	    return PICOSAT_SATISFIABLE;

	  assert (!level);
#endif
	}

      if (need_to_reduce ())
	reduce (50);

      if (conflicts >= lrestart && level > 2)
	restart ();

      decide ();
      if (failed_assumption)
	return PICOSAT_UNSATISFIABLE;
      count++;
    }
}

static void
rebias (void)
{
  Cls ** p, * c;
  Var * v;

  for (v = vars + 1; v <= vars + max_var; v++)
    v->assigned = 0;

  memset (jwh, 0, 2 * (max_var + 1) * sizeof *jwh);

  for (p = oclauses; p < ohead; p++) 
    {
      c = *p;

      if (!c) 
	continue;

      if (c->learned)
	continue;

      incjwh (c);
    }
}

#ifdef TRACE

static unsigned
core (void)
{
  unsigned idx, prev, this, delta, i, lcore, vcore;
  unsigned *stack, *shead, *eos;
  Lit **q, **eol, *lit;
  Cls *cls, *reason;
  Znt *p, byte;
  Zhn *zhain;
  Var *v;

  assert (trace);

  assert (mtcls || failed_assumption);
  if (ocore >= 0)
    return ocore;

  lcore = ocore = vcore = 0;

  stack = shead = eos = 0;
  ENLARGE (stack, shead, eos);

  if (mtcls)
    {
      idx = CLS2IDX (mtcls);
      *shead++ = idx;
    }
  else
    {
      assert (failed_assumption);
      v = LIT2VAR (failed_assumption);
      reason = v->reason;
      assert (reason);
      idx = CLS2IDX (reason);
      *shead++ = idx;
    }

  while (shead > stack)
    {
      idx = *--shead;
      zhain = IDX2ZHN (idx);

      if (zhain)
	{
	  if (zhain->core)
	    continue;

	  zhain->core = 1;
	  lcore++;

	  cls = IDX2CLS (idx);
	  if (cls)
	    {
	      assert (!cls->core);
	      cls->core = 1;
	    }

	  i = 0;
	  delta = 0;
	  prev = 0;
	  for (p = zhain->znt; (byte = *p); p++, i += 7)
	    {
	      delta |= (byte & 0x7f) << i;
	      if (byte & 0x80)
		continue;

	      this = prev + delta;
	      assert (prev < this);	/* no overflow */

	      if (shead == eos)
		ENLARGE (stack, shead, eos);
	      *shead++ = this;

	      prev = this;
	      delta = 0;
	      i = -7;
	    }
	}
      else
	{
	  cls = IDX2CLS (idx);

	  assert (cls);
	  assert (!cls->learned);

	  if (cls->core)
	    continue;

	  cls->core = 1;
	  ocore++;

	  eol = end_of_lits (cls);
	  for (q = cls->lits; q < eol; q++)
	    {
	      lit = *q;
	      v = LIT2VAR (lit);
	      if (v->core)
		continue;

	      v->core = 1;
	      vcore++;

	      if (!failed_assumption) continue;
	      if (lit != failed_assumption) continue;

	      reason = v->reason;
	      if (!reason) continue;
	      if (reason->core) continue;

	      idx = CLS2IDX (reason);
	      if (shead == eos)
		ENLARGE (stack, shead, eos);
	      *shead++ = idx;
	    }
	}
    }

  DELETEN (stack, eos - stack);

  if (verbosity)
    fprintf (out,
	     "%s%u core variables out of %u (%.1f%%)\n"
	     "%s%u core original clauses out of %u (%.1f%%)\n"
	     "%s%u core learned clauses out of %u (%.1f%%)\n",
	     prefix, vcore, max_var, PERCENT (vcore, max_var),
	     prefix, ocore, oadded, PERCENT (ocore, oadded),
	     prefix, lcore, ladded, PERCENT (lcore, ladded));

  return ocore;
}

static void
trace_lits (Cls * cls, FILE * file)
{
  Lit **p, **eol = end_of_lits (cls);

  assert (cls);
  assert (cls->core);

  for (p = cls->lits; p < eol; p++)
    fprintf (file, "%d ", LIT2INT (*p));

  fputc ('0', file);
}

static void
write_idx (unsigned idx, FILE * file)
{
  fprintf (file, "%ld", EXPORTIDX (idx));
}

static void
trace_clause (unsigned idx, Cls * cls, FILE * file, int fmt)
{
  assert (cls);
  assert (cls->core);
  assert (fmt == RUP_TRACE_FMT || !cls->learned);
  assert (CLS2IDX (cls) == idx);

  if (fmt != RUP_TRACE_FMT)
    {
      write_idx (idx, file);
      fputc (' ', file);
    }

  trace_lits (cls, file);

  if (fmt != RUP_TRACE_FMT)
    fputs (" 0", file);

  fputc ('\n', file);
}

static void
trace_zhain (unsigned idx, Zhn * zhain, FILE * file, int fmt)
{
  unsigned prev, this, delta, i;
  Znt *p, byte;
  Cls * cls;

  assert (zhain);
  assert (zhain->core);

  write_idx (idx, file);
  fputc (' ', file);

  if (fmt == EXTENDED_TRACECHECK_TRACE_FMT)
    {
      cls = IDX2CLS (idx);
      assert (cls);
      trace_lits (cls, file);
    }
  else
    {
      assert (fmt == COMPACT_TRACECHECK_TRACE_FMT);
      putc ('*', file);
    }

  i = 0;
  delta = 0;
  prev = 0;

  for (p = zhain->znt; (byte = *p); p++, i += 7)
    {
      delta |= (byte & 0x7f) << i;
      if (byte & 0x80)
	continue;

      this = prev + delta;

      putc (' ', file);
      write_idx (this, file);

      prev = this;
      delta = 0;
      i = -7;
    }

  fputs (" 0\n", file);
}

static void
write_core (FILE * file)
{
  Lit **q, **eol;
  Cls **p, *cls;

  fprintf (file, "p cnf %u %u\n", max_var, core ());

  for (p = SOC; p != EOC; p = NXC (p))
    {
      cls = *p;

      if (!cls || cls->learned || !cls->core)
	continue;

      eol = end_of_lits (cls);
      for (q = cls->lits; q < eol; q++)
	fprintf (file, "%d ", LIT2INT (*q));

      fputs ("0\n", file);
    }
}

#endif

static void
write_trace (FILE * file, int fmt)
{
#ifdef TRACE
  Cls *cls, ** p;
  Zhn *zhain;
  unsigned i;

  core ();

  if (fmt == RUP_TRACE_FMT)
    {
      rupvariables = picosat_variables (),
      rupclauses = picosat_added_original_clauses ();
      write_rup_header (file);
    }

  for (p = SOC; p != EOC; p = NXC (p))
    {
      cls = *p;

      if (oclauses <= p && p < eoo)
	{
	  i = OIDX2IDX (p - oclauses);
	  assert (!cls || CLS2IDX (cls) == i);
	}
      else
	{
          assert (lclauses <= p && p < eol);
	  i = LIDX2IDX (p - lclauses);
	}

      zhain = IDX2ZHN (i);

      if (zhain)
	{
	  if (zhain->core)
	    {
	      if (fmt == RUP_TRACE_FMT)
		trace_clause (i, cls, file, fmt);
	      else
		trace_zhain (i, zhain, file, fmt);
	    }
	}
      else if (cls)
	{
	  if (fmt != RUP_TRACE_FMT && cls)
	    {
	      if (cls->core)
		trace_clause (i, cls, file, fmt);
	    }
	}
    }
#else
  (void) file;
  (void) fmt;
#endif
}

static void
write_core_wrapper (FILE * file, int fmt)
{
  (void) fmt;
#ifdef TRACE
  write_core (file);
#else
  (void) file;
#endif
}

static Lit *
import_lit (int lit, int nointernal)
{
  Lit * res;
  Var * v;

  ABORTIF (lit == INT_MIN, "API usage: INT_MIN literal");
  ABORTIF (abs (lit) > (int) max_var && cls != clshead,
           "API usage: new variable index after 'picosat_push'");

  if (abs (lit) <= (int) max_var)
    {
      res = int2lit (lit);
      v = LIT2VAR (res);
      if (nointernal && v->internal)
	ABORT ("API usage: trying to import invalid literal");
      else if (!nointernal && !v->internal)
	ABORT ("API usage: trying to import invalid context");
    }
  else
    {
      while (abs (lit) > (int) max_var)
	inc_max_var ();
      res = int2lit (lit);
    }

  return res;
}

#ifdef TRACE
static void
reset_core (void)
{
  Cls ** p, * c;
  Zhn ** q, * z;
  unsigned i;

  for (i = 1; i <= max_var; i++)
    vars[i].core = 0;

  for (p = SOC; p != EOC; p = NXC (p))
    if ((c = *p))
      c->core = 0;

  for (q = zhains; q != zhead; q++)
    if ((z = *q))
      z->core = 0;

  ocore = -1;
}
#endif

static void
reset_assumptions (void)
{
  Lit ** p;

  failed_assumption = 0;

  if (extracted_all_failed_assumptions)
    {
      for (p = als; p < alshead; p++)
	LIT2VAR (*p)->failed = 0;

      extracted_all_failed_assumptions = 0;
    }

  alstail = alshead = als;
  adecidelevel = 0;
}

static void
check_ready (void)
{
  ABORTIF (state == RESET, "API usage: uninitialized");
}

static void
check_sat_state (void)
{
  ABORTIF (state != SAT, "API usage: expected to be in SAT state");
}

static void
check_unsat_state (void)
{
  ABORTIF (state != UNSAT, "API usage: expected to be in UNSAT state");
}

static void
check_sat_or_unsat_or_unknown_state (void)
{
  ABORTIF (state != SAT && state != UNSAT && state != UNKNOWN,
           "API usage: expected to be in SAT, UNSAT, or UNKNOWN state");
}

static void
reset_partial (void)
{
  unsigned idx;
  if (!partial)
    return;
  for (idx = 1; idx <= max_var; idx++)
    vars[idx].partial = 0;
  partial = 0;
}

static void
reset_incremental_usage (void)
{
  unsigned num_non_false;
  Lit * lit, ** q;

  check_sat_or_unsat_or_unknown_state ();

  LOG (fprintf (out, "%sRESET incremental usage\n", prefix));

  if (level)
    undo (0);

  reset_assumptions ();

  if (conflict)
    { 
      num_non_false = 0;
      for (q = conflict->lits; q < end_of_lits (conflict); q++)
	{
	  lit = *q;
	  if (lit->val != FALSE)
	    num_non_false++;
	}

      // assert (num_non_false >= 2); // TODO: why this assertion?
#ifdef NO_BINARY_CLAUSES
      if (conflict == &cimpl)
	resetcimpl ();
#endif
#ifndef NADC
      if (conflict == adoconflict)
	resetadoconflict ();
#endif
      conflict = 0;
    }

#ifdef TRACE
  reset_core ();
#endif

  reset_partial ();

  saved_flips = flips;
  min_flipped = UINT_MAX;
  saved_max_var = max_var;

  state = READY;
}

static void
enter (void)
{
  if (nentered++)
    return;

  check_ready ();
  entered = picosat_time_stamp ();
}

static void
leave (void)
{
  assert (nentered);
  if (--nentered)
    return;

  sflush ();
}

static void
check_trace_support_and_execute (FILE * file, void (*f)(FILE*,int), int fmt)
{
  check_ready ();
  check_unsat_state ();
#ifdef TRACE
  ABORTIF (!trace, "API usage: tracing disabled");
  enter ();
  f (file, fmt);
  leave ();
#else
  (void) file;
  (void) fmt;
  (void) f;
  ABORT ("compiled without trace support");
#endif
}

static void
extract_all_failed_assumptions (void)
{
  Lit ** p, ** eol;
  Var * v, * u;
  int pos;
  Cls * c;

  assert (!extracted_all_failed_assumptions);

  assert (failed_assumption);
  assert (mhead == marked);

  if (marked == eom)
    ENLARGE (marked, mhead, eom);

  v = LIT2VAR (failed_assumption);
  mark_var (v);
  pos = 0;

  while (pos < mhead - marked)
    {
      v = marked[pos++];
      assert (v->mark);
      c = var2reason (v);
      if (!c)
	continue;
      eol = end_of_lits (c);
      for (p = c->lits; p < eol; p++)
	{
	  u = LIT2VAR (*p);
	  if (!u->mark)
	    mark_var (u);
	}
#ifdef NO_BINARY_CLAUSES
      if (c == &impl)
	resetimpl ();
#endif
    }

  for (p = als; p < alshead; p++)
    {
      u = LIT2VAR (*p);
      if (!u->mark) continue;
      u->failed = 1;
      LOG (fprintf (out, "%sfailed assumption %d\n", prefix, LIT2INT (*p)));
    }

  while (mhead > marked)
    (*--mhead)->mark = 0;

  extracted_all_failed_assumptions = 1;
}

const char *
picosat_copyright (void)
{
  return "Copyright (c) 2006 - 2012 Armin Biere JKU Linz";
}

void
picosat_init (void)
{
  init ();
}

void
picosat_adjust (int new_max_var)
{
  unsigned new_size_vars;

  ABORTIF (abs (new_max_var) > (int) max_var && cls != clshead,
           "API usage: adjusting variable index after 'picosat_push'");
  enter ();

  new_max_var = abs (new_max_var);
  new_size_vars = new_max_var + 1;

  if (size_vars < new_size_vars)
    enlarge (new_size_vars);

  while (max_var < (unsigned) new_max_var)
    inc_max_var ();

  leave ();
}

int
picosat_inc_max_var (void)
{
  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  inc_max_var ();

  if (measurealltimeinlib)
    leave ();

  return max_var;
}

int
picosat_context (void)
{
  return clshead == cls ? 0 : LIT2INT (clshead[-1]);
}

int
picosat_push (void)
{
  int res;
  Lit *lit;
  Var * v;

  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  if (state != READY)
    reset_incremental_usage ();

  if (rils != rilshead)
    {
      res = *--rilshead;
      assert (vars[res].internal);
    }
  else
    {
      inc_max_var ();
      res = max_var;
      v = vars + res;
      assert (!v->internal);
      v->internal = 1;
      internals++;
      LOG (fprintf (out, "%snew internal variable index %d\n", prefix, res));
    }

  lit = int2lit (res);
  if (clshead == eocls)
    ENLARGE (cls, clshead, eocls);

  *clshead++ = lit;
  contexts++;

  LOG (fprintf (out, "%snew context %d at depth %ld after push\n",
                prefix, res, (long)(clshead - cls)));

  if (measurealltimeinlib)
    leave ();

  return res;
}

int 
picosat_pop (void)
{
  Lit * lit;
  int res;
  ABORTIF (cls == clshead, "API usage: too many 'picosat_pop'");
  ABORTIF (added != ahead, "API usage: incomplete clause");

  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  if (state != READY)
    reset_incremental_usage ();

  assert (cls < clshead);
  lit = *--clshead;
  LOG (fprintf (out, "%sclosing context %d at depth %ld after pop\n",
                prefix, LIT2INT (lit), (long)(clshead - cls) + 1));

  if (cilshead == eocils)
    ENLARGE (cils, cilshead, eocils);
  *cilshead++ = LIT2INT (lit);

  if (cilshead - cils > MAXCILS) {
    LOG (fprintf (out,
                  "%srecycling %ld interals with forced simplification\n",
		  prefix, (long)(cilshead - cils)));
    simplify (1);
  }

  res = picosat_context ();
  if (res)
    LOG (fprintf (out, "%snew context %d at depth %ld after pop\n",
		  prefix, res, (long)(clshead - cls)));
  else
    LOG (fprintf (out, "%souter most context reached after pop\n", prefix));

  if (measurealltimeinlib)
    leave ();
  
  return res;
}

void
picosat_set_verbosity (int new_verbosity_level)
{
  check_ready ();
  verbosity = new_verbosity_level;
}

int
picosat_enable_trace_generation (void)
{
  int res = 0;
  check_ready ();
#ifdef TRACE
  ABORTIF (addedclauses, 
           "API usage: trace generation enabled after adding clauses");
  res = trace = 1;
#endif
  return res;
}

void
picosat_set_incremental_rup_file (FILE * rup_file, int m, int n)
{
  check_ready ();
  assert (!rupstarted);
  rup = rup_file;
  rupvariables = m;
  rupclauses = n;
}

void
picosat_set_output (FILE * output_file)
{
  check_ready ();
  out = output_file;
}

void
picosat_measure_all_calls (void)
{
  check_ready ();
  measurealltimeinlib = 1;
}

void
picosat_set_prefix (const char * str)
{
  check_ready ();
  new_prefix (str);
}

void
picosat_set_seed (unsigned s)
{
  check_ready ();
  srng = s;
}

void
picosat_reset (void)
{
  check_ready ();
  reset ();
}

int
picosat_add (int int_lit)
{
  int res = oadded;
  Lit *lit;

  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  ABORTIF (rup && rupstarted && oadded >= (unsigned)rupclauses,
           "API usage: adding too many clauses after RUP header written");
#ifndef NADC
  ABORTIF (addingtoado, 
           "API usage: 'picosat_add' and 'picosat_add_ado_lit' mixed");
#endif
  if (state != READY)
    reset_incremental_usage ();

  if (saveorig)
    {
      if (sohead == eoso)
	ENLARGE (soclauses, sohead, eoso);

      *sohead++ = int_lit;
    }

  if (int_lit)
    {
      lit = import_lit (int_lit, 1);
      add_lit (lit);
    }
  else
    simplify_and_add_original_clause ();

  if (measurealltimeinlib)
    leave ();

  return res;
}

void
picosat_add_ado_lit (int external_lit)
{
#ifndef NADC
  Lit * internal_lit;

  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  if (state != READY)
    reset_incremental_usage ();

  ABORTIF (!addingtoado && ahead > added,
           "API usage: 'picosat_add' and 'picosat_add_ado_lit' mixed");

  if (external_lit)
    {
      addingtoado = 1;
      internal_lit = import_lit (external_lit, 1);
      add_lit (internal_lit);
    }
  else
    {
      addingtoado = 0;
      add_ado ();
    }
  if (measurealltimeinlib)
    leave ();
#else
  (void) external_lit;
  ABORT ("compiled without all different constraint support");
#endif
}

static void
assume (Lit * lit)
{
  if (alshead == eoals)
    {
      assert (alstail == als);
      ENLARGE (als, alshead, eoals);
      alstail = als;
    }

  *alshead++ = lit;
  LOG (fprintf (out, "%sassumption %d\n", prefix, LIT2INT (lit)));
}

static void
assume_contexts (void)
{
  Lit ** p;
  if (als != alshead)
    return;
  for (p = cls; p != clshead; p++)
    assume (*p);
}

static const char * enumstr (int i) {
  int last = i % 10;
  if (last == 1) return "st";
  if (last == 2) return "nd";
  if (last == 3) return "rd";
  return "th";
}

static void
setemgr (void * nmgr)
{
  ABORTIF (emgr && emgr != nmgr, 
           "API usage: mismatched external memory managers");
  emgr = nmgr;
}

static int
pderef (int int_lit)
{
  Lit * lit;
  Var * v;

  if (abs (int_lit) > (int) max_var)
    return 0;

  v = vars + abs (int_lit);
  
  if (!v->partial)
    return 0;

  lit = int2lit (int_lit);

  if (lit->val == TRUE)
    return 1;

  if (lit->val == FALSE)
    return -1;

  return 0;
}

static void
minautarky (void)
{
  unsigned * occs, maxoccs, tmpoccs, npartial;
  int * p, * c, lit, best, val;

  assert (!partial);

  npartial = 0;

  NEWN (occs, 2*max_var + 1);
  occs += max_var;
  for (p = soclauses; p < sohead; p++)
    occs[*p]++;
  assert (occs[0] == oadded);

  for (c = soclauses; c < sohead; c = p + 1) 
    {
      best = 0; 
      maxoccs = 0;
      for (p = c; (lit = *p); p++)
	{
	  val = pderef (lit);
	  if (val > 0)
	    break;
	  if (val < 0)
	    continue;
	  tmpoccs = occs[lit];
	  if (best && tmpoccs <= maxoccs)
	    continue;
	  best = lit;
	  maxoccs = tmpoccs;
	}
      if (!lit)
	{
	  assert (best);
	  LOG (fprintf (out, "%sautark %d with %d occs\n", 
	       prefix, best, maxoccs));
	  vars[abs (best)].partial = 1;
	  npartial++;
	}
      for (p = c; (lit = *p); p++)
	{
	  val = pderef (lit);
	  if (val)
	    continue;
	  assert (occs[lit] > 0);
	  occs[lit]--;
	}
    }
  occs -= max_var;
  DELETEN (occs, 2*max_var + 1);
  partial = 1;

  if (verbosity)
    fprintf (out,
      "%sautarky of size %u out of %u satisfying all clauses (%.1f%%)\n",
      prefix, npartial, max_var, PERCENT (npartial, max_var));
}

void
picosat_assume (int int_lit)
{
  Lit *lit;

  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  if (state != READY)
    reset_incremental_usage ();

  assume_contexts ();
  lit = import_lit (int_lit, 1);
  assume (lit);

  if (measurealltimeinlib)
    leave ();
}

int
picosat_sat (int l)
{
  int res;
  char ch;

  enter ();

  calls++;
  LOG (fprintf (out, "%sSTART call %u\n", prefix, calls));

  if (added < ahead)
    {
#ifndef NADC
      if (addingtoado)
	ABORT ("API usage: incomplete all different constraint");
      else
#endif
	ABORT ("API usage: incomplete clause");
    }

  if (state != READY)
    reset_incremental_usage ();

  assume_contexts ();

  res = sat (l);

  assert (state == READY);

  switch (res)
    {
    case PICOSAT_UNSATISFIABLE:
      ch = '0';
      state = UNSAT;
      break;
    case PICOSAT_SATISFIABLE:
      ch = '1';
      state = SAT;
      break;
    default:
      ch = '?';
      state = UNKNOWN;
      break;
    }

  if (verbosity)
    {
      report (1, ch);
      rheader ();
    }

  leave ();
  LOG (fprintf (out, "%sEND call %u result %d\n", prefix, calls, res));

  last_sat_call_result = res;

  return res;
}

int
picosat_res (void)
{
  return last_sat_call_result;
}

int
picosat_deref (int int_lit)
{
  Lit *lit;

  check_ready ();
  check_sat_state ();
  ABORTIF (!int_lit, "API usage: can not deref zero literal");
  ABORTIF (mtcls, "API usage: deref after empty clause generated");

#ifdef STATS
  derefs++;
#endif

  if (abs (int_lit) > (int) max_var)
    return 0;

  lit = int2lit (int_lit);

  if (lit->val == TRUE)
    return 1;

  if (lit->val == FALSE)
    return -1;

  return 0;
}

int
picosat_deref_toplevel (int int_lit)
{
  Lit *lit;
  Var * v;

  check_ready ();
  ABORTIF (!int_lit, "API usage: can not deref zero literal");
  ABORTIF (mtcls, "API usage: deref after empty clause generated");

#ifdef STATS
  derefs++;
#endif
  if (abs (int_lit) > (int) max_var)
    return 0;

  lit = int2lit (int_lit);

  v = LIT2VAR (lit);
  if (v->level > 0)
    return 0;

  if (lit->val == TRUE)
    return 1;

  if (lit->val == FALSE)
    return -1;

  return 0;
}

int
picosat_inconsistent (void)
{
  check_ready ();
  return mtcls != 0;
}

int
picosat_corelit (int int_lit)
{
  check_ready ();
  check_unsat_state ();
  ABORTIF (!int_lit, "API usage: zero literal can not be in core");

  assert (mtcls || failed_assumption);

#ifdef TRACE
  {
    int res = 0;
    ABORTIF (!trace, "tracing disabled");
    if (measurealltimeinlib)
      enter ();
    core ();
    if (abs (int_lit) <= (int) max_var)
      res = vars[abs (int_lit)].core;
    assert (!res || failed_assumption || vars[abs (int_lit)].used);
    if (measurealltimeinlib)
      leave ();
    return res;
  }
#else
  ABORT ("compiled without trace support");
  return 0;
#endif
}

int
picosat_coreclause (int ocls)
{
  check_ready ();
  check_unsat_state ();

  ABORTIF (ocls < 0, "API usage: negative original clause index");
  ABORTIF (ocls >= (int)oadded, "API usage: original clause index exceeded");

  assert (mtcls || failed_assumption);

#ifdef TRACE
  {
    Cls ** clsptr, * cls;
    int res  = 0;

    ABORTIF (!trace, "tracing disabled");
    if (measurealltimeinlib)
      enter ();
    core ();
    clsptr = oclauses + ocls;
    assert (clsptr < ohead);
    cls = *clsptr;
    if (cls) 
      res = cls->core;
    if (measurealltimeinlib)
      leave ();

    return res;
  }
#else
  ABORT ("compiled without trace support");
  return 0;
#endif
}

int
picosat_failed_assumption (int int_lit)
{
  Lit * lit;
  Var * v;
  ABORTIF (!int_lit, "API usage: zero literal as assumption");
  check_ready ();
  check_unsat_state ();
  if (mtcls)
    return 0;
  assert (failed_assumption);
  if (abs (int_lit) > (int) max_var)
    return 0;
  if (!extracted_all_failed_assumptions)
    extract_all_failed_assumptions ();
  lit = import_lit (int_lit, 1);
  v = LIT2VAR (lit);
  return v->failed;
}

int
picosat_failed_context (int int_lit)
{
  Lit * lit;
  Var * v;
  ABORTIF (!int_lit, "API usage: zero literal as context");
  ABORTIF (abs (int_lit) > (int) max_var, "API usage: invalid context");
  check_ready ();
  check_unsat_state ();
  assert (failed_assumption);
  if (!extracted_all_failed_assumptions)
    extract_all_failed_assumptions ();
  lit = import_lit (int_lit, 0);
  v = LIT2VAR (lit);
  return v->failed;
}

const int *
picosat_failed_assumptions (void)
{
  Lit ** p, * lit;
  Var * v;
  int ilit;

  falshead = fals;
  check_ready ();
  check_unsat_state ();
  if (!mtcls) 
    {
      assert (failed_assumption);
      if (!extracted_all_failed_assumptions)
	extract_all_failed_assumptions ();

      for (p = als; p < alshead; p++)
	{
	  lit = *p;
	  v = LIT2VAR (*p);
	  if (!v->failed)
	    continue;
	  ilit = LIT2INT (lit);
	  if (falshead == eofals)
	    ENLARGE (fals, falshead, eofals);
	  *falshead++ = ilit;
	}
    }
  if (falshead == eofals)
    ENLARGE (fals, falshead, eofals);
  *falshead++ = 0;
  return fals;
}

const int *
picosat_mus_assumptions (void * s, void (*cb)(void*,const int*), int fix)
{
  int i, j, ilit, len, norig = alshead - als, nwork, * work, res;
  signed char * redundant;
  Lit ** p, * lit;
  int failed;
  Var * v;
#ifndef NDEBUG
  int oldlen;
#endif

  check_ready ();
  check_unsat_state ();
  len = 0;
  if (!mtcls) 
    {
      assert (failed_assumption);
      if (!extracted_all_failed_assumptions)
	extract_all_failed_assumptions ();

      for (p = als; p < alshead; p++)
	if (LIT2VAR (*p)->failed)
	  len++;
    }

  if (mass)
    DELETEN (mass, szmass);
  szmass = len + 1;
  NEWN (mass, szmass);

  i = 0;
  for (p = als; p < alshead; p++)
    {
      lit = *p;
      v = LIT2VAR (lit);
      if (!v->failed)
	continue;
      ilit = LIT2INT (lit);
      assert (i < len);
      mass[i++] = ilit;
    }
  assert (i == len);
  mass[i] = 0;
  if (verbosity)
    fprintf (out, 
      "%sinitial set of failed assumptions of size %d out of %d (%.0f%%)\n",
      prefix, len, norig, PERCENT (len, norig));
  if (cb)
    cb (s, mass);

  nwork = len;
  NEWN (work, nwork);
  for (i = 0; i < len; i++)
    work[i] = mass[i];

  NEWN (redundant, nwork);
  CLRN (redundant, nwork);

  for (i = 0; i < nwork; i++)
    {
      if (redundant[i])
	continue;

      if (verbosity > 1)
	fprintf (out,
	         "%strying to drop %d%s assumption %d\n", 
		 prefix, i, enumstr (i), work[i]);
      for (j = 0; j < nwork; j++)
	{
	  if (i == j) continue;
	  if (j < i && fix) continue;
	  if (redundant[j]) continue;
	  picosat_assume (work[j]);
	}

      res = picosat_sat (-1);
      if (res == 10)
	{
	  if (verbosity > 1)
	    fprintf (out,
		     "%sfailed to drop %d%s assumption %d\n", 
		     prefix, i, enumstr (i), work[i]);

	  if (fix)
	    {
	      picosat_add (work[i]);
	      picosat_add (0);
	    }
	}
      else
	{
	  assert (res == 20);
	  if (verbosity > 1)
	    fprintf (out,
		     "%ssuceeded to drop %d%s assumption %d\n", 
		     prefix, i, enumstr (i), work[i]);
	  redundant[i] = 1;
	  for (j = 0; j < nwork; j++)
	    {
	      failed = picosat_failed_assumption (work[j]);
	      if (j <= i) 
		{
		  assert ((j < i && fix) || redundant[j] == !failed);
		  continue;
		}

	      if (!failed)
		{
		  redundant[j] = -1;
		  if (verbosity > 1)
		    fprintf (out,
			     "%salso suceeded to drop %d%s assumption %d\n", 
			     prefix, j, enumstr (j), work[j]);
		}
	    }

#ifndef NDEBUG
	    oldlen = len;
#endif
	    len = 0;
	    for (j = 0; j < nwork; j++)
	      if (!redundant[j])
		mass[len++] = work[j];
	    mass[len] = 0;
	    assert (len < oldlen);

	    if (fix)
	      {
		picosat_add (-work[i]);
		picosat_add (0);
	      }

#ifndef NDEBUG
	    for (j = 0; j <= i; j++)
	      assert (redundant[j] >= 0);
#endif
	    for (j = i + 1; j < nwork; j++) 
	      {
		if (redundant[j] >= 0)
		  continue;

		if (fix)
		  {
		    picosat_add (-work[j]);
		    picosat_add (0);
		  }

		redundant[j] = 1;
	      }

	    if (verbosity)
	      fprintf (out, 
	"%sreduced set of failed assumptions of size %d out of %d (%.0f%%)\n",
		prefix, len, norig, PERCENT (len, norig));
	    if (cb)
	      cb (s, mass);
	}
    }

  DELETEN (work, nwork);
  DELETEN (redundant, nwork);

  if (verbosity)
    {
      fprintf (out, "%sreinitializing unsat state\n", prefix);
      fflush (out);
    }

  for (i = 0; i < len; i++)
    picosat_assume (mass[i]);

#ifndef NDEBUG
  res = 
#endif
  picosat_sat (-1);
  assert (res == 20);

  if (!mtcls)
    {
      assert (!extracted_all_failed_assumptions);
      extract_all_failed_assumptions ();
    }

  return mass;
}

static const int *
mss (int * a, int size)
{
  int i, j, k, res;

  assert (!mtcls);

  if (szmssass)
    DELETEN (mssass, szmssass);

  szmssass = 0;
  mssass = 0;

  szmssass = size + 1;
  NEWN (mssass, szmssass);

  LOG (fprintf (out, "%ssearch MSS over %d assumptions\n", prefix, size));

  k = 0;
  for (i = k; i < size; i++)
    {
      for (j = 0; j < k; j++)
	picosat_assume (mssass[j]);

      LOG (fprintf (out, 
             "%strying to add assumption %d to MSS : %d\n", 
	     prefix, i, a[i])); 

      picosat_assume (a[i]);

      res = picosat_sat (-1);
      if (res == 10)
	{
	  LOG (fprintf (out, 
		 "%sadding assumption %d to MSS : %d\n", prefix, i, a[i])); 

	  mssass[k++] = a[i];

	  for (j = i + 1; j < size; j++)
	    {
	      if (picosat_deref (a[j]) <= 0)
		continue;

	      LOG (fprintf (out, 
		     "%salso adding assumption %d to MSS : %d\n", 
		     prefix, j, a[j])); 

	      mssass[k++] = a[j];

	      if (++i != j)
		{
		  int tmp = a[i];
		  a[i] = a[j];
		  a[j] = tmp;
		}
	    }
	}
      else
	{
	  assert (res == 20);

	  LOG (fprintf (out, 
		 "%signoring assumption %d in MSS : %d\n", prefix, i, a[i])); 
	}
    }
  mssass[k] = 0;
  LOG (fprintf (out, "%sfound MSS of size %d\n", prefix, k));

  return mssass;
}

static void
reassume (const int * a, int size)
{
  int i;
  LOG (fprintf (out, "%sreassuming all assumptions\n", prefix));
  for (i = 0; i < size; i++)
    picosat_assume (a[i]);
}

const int *
picosat_maximal_satisfiable_subset_of_assumptions (void)
{
  const int * res;
  int i, *a, size;

  ABORTIF (mtcls, "API usage: CNF inconsistent (use 'picosat_inconsistent')");

  enter ();

  size = alshead - als;
  NEWN (a, size);

  for (i = 0; i < size; i++)
    a[i] = LIT2INT (als[i]);

  res = mss (a, size);
  reassume (a, size);

  DELETEN (a, size);

  leave ();

  return res;
}

static void
check_mss_flags_clean (void)
{
#ifndef NDEBUG
  unsigned i;
  for (i = 1; i <= max_var; i++)
    {
      assert (!vars[i].msspos);
      assert (!vars[i].mssneg);
    }
#endif
}

static void
push_mcsass (int lit)
{
  if (nmcsass == szmcsass)
    {
      szmcsass = szmcsass ? 2*szmcsass : 1;
      RESIZEN (mcsass, nmcsass, szmcsass);
    }

  mcsass[nmcsass++] = lit;
}

static const int *
next_mss (int mcs)
{
  int i, *a, size, mssize, mcsize, lit, inmss;
  const int * res, * p;
  Var * v;

  if (mtcls) return 0;

  check_mss_flags_clean ();

  if (mcs && mcsass)
    {
      DELETEN (mcsass, szmcsass);
      nmcsass = szmcsass = 0;
      mcsass = 0;
    }

  size = alshead - als;
  NEWN (a, size);

  for (i = 0; i < size; i++)
    a[i] = LIT2INT (als[i]);

  (void) picosat_sat (-1);

  //TODO short cut for 'picosat_res () == 10'?

  if (mtcls)
    {
      assert (picosat_res () == 20);
      res = 0;
      goto DONE;
    }

  res = mss (a, size);

  if (mtcls)
    {
      res = 0;
      goto DONE;
    }

  for (p = res; (lit = *p); p++) 
    {
      v = vars + abs (lit);
      if (lit < 0)
	{
	  assert (!v->msspos);
	  v->mssneg = 1;
	}
      else
	{
	  assert (!v->mssneg);
	  v->msspos = 1;
	}
    }

  mssize = p - res;
  mcsize = INT_MIN;

  for (i = 0; i < size; i++)
    {
      lit = a[i];
      v = vars + abs (lit);
      if (lit > 0 && v->msspos)
	inmss = 1;
      else if (lit < 0 && v->mssneg)
	inmss = 1;
      else 
	inmss = 0;

      if (mssize < mcsize)
	{
	  if (inmss)
	    picosat_add (-lit);
	}
      else
	{
	  if (!inmss)
	    picosat_add (lit);
	}

      if (!inmss && mcs)
	push_mcsass (lit);
    }
  picosat_add (0);
  if (mcs)
    push_mcsass (0);

  for (i = 0; i < size; i++)
    {
      lit = a[i];
      v = vars + abs (lit);
      v->msspos = 0;
      v->mssneg = 0;
    }

DONE:

  reassume (a, size);
  DELETEN (a, size);

  return res;
}

const int *
picosat_next_maximal_satisfiable_subset_of_assumptions (void)
{
  const int * res;
  enter ();
  res = next_mss (0);
  leave ();
  return  res;
}

const int *
picosat_next_minimal_correcting_subset_of_assumptions (void)
{
  const int * res, * tmp;
  enter ();
  tmp = next_mss (1);
  res = tmp ? mcsass : 0;
  leave ();
  return res;
}

const int *
picosat_humus (void (*callback)(void*state,int nmcs,int nhumus), void * state)
{
  int lit, nmcs, j, nhumus;
  const int * mcs, * p;
  unsigned i;
  Var * v;
  enter ();
#ifndef NDEBUG
  for (i = 1; i <= max_var; i++)
    {
      v = vars + i;
      assert (!v->humuspos);
      assert (!v->humusneg);
    }
#endif
  nhumus = nmcs = 0;
  while ((mcs = picosat_next_minimal_correcting_subset_of_assumptions ()))
    {
      for (p = mcs; (lit = *p); p++)
	{
	  v = vars + abs (lit);
	  if (lit < 0)
	    {
	      if (!v->humusneg)
		{
		  v->humusneg = 1;
		  nhumus++;
		}
	    }
	  else
	    {
	      if (!v->humuspos)
		{
		  v->humuspos = 1;
		  nhumus++;
		}
	    }
	}
      nmcs++;
      LOG (fprintf (out, 
             "%smcs %d of size %d humus %d\n",
	     prefix, nmcs, (int)(p - mcs), nhumus));
      if (callback)
	callback (state, nmcs, nhumus);
    }
  assert (!szhumus);
  szhumus = 1;
  for (i = 1; i <= max_var; i++)
    {
      v = vars + i;
      if (v->humuspos)
	szhumus++;
      if (v->humusneg)
	szhumus++;
    }
  assert (nhumus + 1 == szhumus);
  NEWN (humus, szhumus);
  j = 0;
  for (i = 1; i <= max_var; i++)
    {
      v = vars + i;
      if (v->humuspos)
	{
	  assert (j < nhumus);
	  humus[j++] = (int) i;
	}
      if (v->humusneg)
	{
	  assert (j < nhumus);
	  assert (i < INT_MAX);
	  humus[j++] = - (int) i;
	}
    }
  assert (j == nhumus);
  assert (j < szhumus);
  humus[j] = 0;
  leave ();
  return humus;
}

int
picosat_usedlit (int int_lit)
{
  int res;
  check_ready ();
  check_sat_or_unsat_or_unknown_state ();
  ABORTIF (!int_lit, "API usage: zero literal can not be used");
  int_lit = abs (int_lit);
  res = (int_lit <= (int) max_var) ? vars[int_lit].used : 0;
  return res;
}

void
picosat_write_clausal_core (FILE * file)
{
  check_trace_support_and_execute (file, write_core_wrapper, 0);
}

void
picosat_write_compact_trace (FILE * file)
{
  check_trace_support_and_execute (file, write_trace,
                                   COMPACT_TRACECHECK_TRACE_FMT);
}

void
picosat_write_extended_trace (FILE * file)
{
  check_trace_support_and_execute (file, write_trace,
                                   EXTENDED_TRACECHECK_TRACE_FMT);
}

void
picosat_write_rup_trace (FILE * file)
{
  check_trace_support_and_execute (file, write_trace, RUP_TRACE_FMT);
}

size_t
picosat_max_bytes_allocated (void)
{
  check_ready ();
  return max_bytes;
}

void
picosat_set_propagation_limit (unsigned long long l)
{
  lpropagations = l;
}

unsigned long long
picosat_propagations (void)
{
  return propagations;
}

unsigned long long
picosat_visits (void)
{
  return visits;
}

unsigned long long
picosat_decisions (void)
{
  return decisions;
}

int
picosat_variables (void)
{
  check_ready ();
  return (int) max_var;
}

int
picosat_added_original_clauses (void)
{
  check_ready ();
  return (int) oadded;
}

void
picosat_stats (void)
{
  unsigned redlits;
#ifdef STATS
  check_ready ();
  assert (sdecisions + rdecisions + assumptions == decisions);
#endif
  if (calls > 1)
    fprintf (out, "%s%u calls\n", prefix, calls);
  if (contexts)
    {
      fprintf (out, "%s%u contexts", prefix, contexts);
#ifdef STATS
      fprintf (out, " %u internal variables", internals);
#endif
      fprintf (out, "\n");
    }
  fprintf (out, "%s%u iterations\n", prefix, iterations);
  fprintf (out, "%s%u restarts", prefix, restarts);
#ifdef STATS
  fprintf (out, " (%u skipped)", skippedrestarts);
#endif
  fputc ('\n', out);
#ifndef NFL
  fprintf (out, "%s%u failed literals", prefix, failedlits);
#ifdef STATS
  fprintf (out,
           ", %u calls, %u rounds, %llu propagations",
           flcalls, flrounds, flprops);
#endif
  fputc ('\n', out);
#ifdef STATS
  fprintf (out, 
    "%sfl: %u = %.1f%% implicit, %llu oopsed, %llu tried, %llu skipped\n", 
    prefix, 
    ifailedlits, PERCENT (ifailedlits, failedlits),
    floopsed, fltried, flskipped);
#endif
#endif
  fprintf (out, "%s%u conflicts", prefix, conflicts);
#ifdef STATS
  fprintf (out, " (%u uips = %.1f%%)\n", uips, PERCENT(uips,conflicts));
#else
  fputc ('\n', out);
#endif
#ifndef NADC
  fprintf (out, "%s%u adc conflicts\n", prefix, adoconflicts);
#endif
#ifdef STATS
  fprintf (out, "%s%llu dereferenced literals\n", prefix, derefs);
#endif
  fprintf (out, "%s%u decisions", prefix, decisions);
#ifdef STATS
  fprintf (out, " (%u random = %.2f%%",
           rdecisions, PERCENT (rdecisions, decisions));
  fprintf (out, ", %u assumptions", assumptions);
  fputc (')', out);
#endif
  fputc ('\n', out);
#ifdef STATS
  fprintf (out,
           "%s%u static phase decisions (%.1f%% of all variables)\n",
	   prefix,
	   staticphasedecisions, PERCENT (staticphasedecisions, max_var));
#endif
  fprintf (out, "%s%u fixed variables\n", prefix, fixed);
  assert (nonminimizedllits >= minimizedllits);
  redlits = nonminimizedllits - minimizedllits;
  fprintf (out, "%s%u learned literals\n", prefix, llitsadded);
  fprintf (out, "%s%.1f%% deleted literals\n",
     prefix, PERCENT (redlits, nonminimizedllits));

#ifdef STATS
#ifndef NO_BINARY_CLAUSES
  fprintf (out,
	   "%s%llu antecedents (%.1f antecedents per clause",
	   prefix, antecedents, AVERAGE (antecedents, conflicts));
#endif
#ifdef TRACE
  if (trace)
    fprintf (out, ", %.1f bytes/antecedent)", AVERAGE (znts, antecedents));
#endif
#if !defined(NO_BINARY_CLAUSES) || defined(TRACE)
  fputs (")\n", out);
#endif

  fprintf (out, "%s%llu propagations (%.1f propagations per decision)\n",
           prefix, propagations, AVERAGE (propagations, decisions));
  fprintf (out, "%s%llu visits (%.1f per propagation)\n",
	   prefix, visits, AVERAGE (visits, propagations));
  fprintf (out, 
           "%s%llu binary clauses visited (%.1f%% %.1f per propagation)\n",
	   prefix, bvisits, 
	   PERCENT (bvisits, visits),
	   AVERAGE (bvisits, propagations));
  fprintf (out, 
           "%s%llu ternary clauses visited (%.1f%% %.1f per propagation)\n",
	   prefix, tvisits, 
	   PERCENT (tvisits, visits),
	   AVERAGE (tvisits, propagations));
  fprintf (out, 
           "%s%llu large clauses visited (%.1f%% %.1f per propagation)\n",
	   prefix, lvisits, 
	   PERCENT (lvisits, visits),
	   AVERAGE (lvisits, propagations));
  fprintf (out, "%s%llu other true (%.1f%% of visited clauses)\n",
	   prefix, othertrue, PERCENT (othertrue, visits));
  fprintf (out, 
           "%s%llu other true in binary clauses (%.1f%%)"
	   ", %llu upper (%.1f%%)\n",
           prefix, othertrue2, PERCENT (othertrue2, othertrue),
	   othertrue2u, PERCENT (othertrue2u, othertrue2));
  fprintf (out, 
           "%s%llu other true in large clauses (%.1f%%)"
	   ", %llu upper (%.1f%%)\n",
           prefix, othertruel, PERCENT (othertruel, othertrue),
	   othertruelu, PERCENT (othertruelu, othertruel));
  fprintf (out, "%s%llu ternary and large traversals (%.1f per visit)\n",
	   prefix, traversals, AVERAGE (traversals, visits));
  fprintf (out, "%s%llu large traversals (%.1f per large visit)\n",
	   prefix, ltraversals, AVERAGE (ltraversals, lvisits));
  fprintf (out, "%s%llu assignments\n", prefix, assignments);
#else
  fprintf (out, "%s%llu propagations\n", prefix, picosat_propagations ());
  fprintf (out, "%s%llu visits\n", prefix, picosat_visits ());
#endif
  fprintf (out, "%s%.1f%% variables used\n", prefix, PERCENT (vused, max_var));

  sflush ();
  fprintf (out, "%s%.1f seconds in library\n", prefix, seconds);
  fprintf (out, "%s%.1f megaprops/second\n",
	   prefix, AVERAGE (propagations / 1e6f, seconds));
  fprintf (out, "%s%.1f megavisits/second\n",
	   prefix, AVERAGE (visits / 1e6f, seconds));
  fprintf (out, "%sprobing %.1f seconds %.0f%%\n",
           prefix, flseconds, PERCENT (flseconds, seconds));
#ifdef STATS
  fprintf (out,
	   "%srecycled %.1f MB in %u reductions\n",
	   prefix, rrecycled / (double) (1 << 20), reductions);
  fprintf (out,
	   "%srecycled %.1f MB in %u simplifications\n",
	   prefix, srecycled / (double) (1 << 20), simps);
#else
  fprintf (out, "%s%u simplifications\n", prefix, simps);
  fprintf (out, "%s%u reductions\n", prefix, reductions);
  fprintf (out, "%s%.1f MB recycled\n", prefix, recycled / (double) (1 << 20));
#endif
  fprintf (out, "%s%.1f MB maximally allocated\n",
	   prefix, picosat_max_bytes_allocated () / (double) (1 << 20));
}

#ifndef NGETRUSAGE
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/unistd.h>
#endif

double
picosat_time_stamp (void)
{
  double res = -1;
#ifndef NGETRUSAGE
  struct rusage u;
  res = 0;
  if (!getrusage (RUSAGE_SELF, &u))
    {
      res += u.ru_utime.tv_sec + 1e-6 * u.ru_utime.tv_usec;
      res += u.ru_stime.tv_sec + 1e-6 * u.ru_stime.tv_usec;
    }
#endif
  return res;
}

double
picosat_seconds (void)
{
  check_ready ();
  return seconds;
}

void
picosat_print (FILE * file)
{
#ifdef NO_BINARY_CLAUSES
  Lit * lit, *other, * last;
  Ltk * stack;
#endif
  Lit **q, **eol;
  Cls **p, *cls;
  unsigned n;

  if (measurealltimeinlib)
    enter ();
  else
    check_ready ();

  n = 0;
  n +=  alshead - als;

  for (p = SOC; p != EOC; p = NXC (p))
    {
      cls = *p;

      if (!cls)
	continue;

#ifdef TRACE
      if (cls->collected)
	continue;
#endif
      n++;
    }

#ifdef NO_BINARY_CLAUSES
  last = int2lit (-max_var);
  for (lit = int2lit (1); lit <= last; lit++)
    {
      stack = LIT2IMPLS (lit);
      eol = stack->start + stack->count;
      for (q = stack->start; q < eol; q++)
	if (*q >= lit)
	  n++;
    }
#endif

  fprintf (file, "p cnf %d %u\n", max_var, n);

  for (p = SOC; p != EOC; p = NXC (p))
    {
      cls = *p;
      if (!cls)
	continue;

#ifdef TRACE
      if (cls->collected)
	continue;
#endif

      eol = end_of_lits (cls);
      for (q = cls->lits; q < eol; q++)
	fprintf (file, "%d ", LIT2INT (*q));

      fputs ("0\n", file);
    }

#ifdef NO_BINARY_CLAUSES
  last = int2lit (-max_var);
  for (lit = int2lit (1); lit <= last; lit++)
    {
      stack = LIT2IMPLS (lit);
      eol = stack->start + stack->count;
      for (q = stack->start; q < eol; q++)
	if ((other = *q) >= lit)
	  fprintf (file, "%d %d 0\n", LIT2INT (lit), LIT2INT (other));
    }
#endif

  {
    Lit **r;
    for (r = als; r < alshead; r++)
      fprintf (file, "%d 0\n", LIT2INT (*r));
  }

  fflush (file);

  if (measurealltimeinlib)
    leave ();
}

void
picosat_enter (void)
{
  enter ();
}

void
picosat_leave (void)
{
  leave ();
}

void
picosat_message (int level, const char * fmt, ...)
{
  va_list ap;

  if (level > verbosity)
    return;

  fputs (prefix, out);
  va_start (ap, fmt);
  vfprintf (out, fmt, ap);
  va_end (ap);
  fputc ('\n', out);
}

int
picosat_changed (void)
{
  int res;

  check_ready ();
  check_sat_state ();

  res = (min_flipped <= saved_max_var);
  assert (!res || saved_flips != flips);

  return res;
}

void
picosat_set_new (void * nmgr, void * (*nnew)(void*,size_t))
{
  ABORTIF (state != RESET, 
           "API usage: 'picosat_set_new' after 'picosat_init'");
  enew = nnew;
  setemgr (nmgr);
}

void
picosat_set_resize (void * nmgr, void * (*nresize)(void*,void*,size_t,size_t))
{
  ABORTIF (state != RESET,
           "API usage: 'picosat_set_resize' after 'picosat_init'");
  eresize = nresize;
  setemgr (nmgr);
}

void
picosat_set_delete (void * nmgr, void (*ndelete)(void*,void*,size_t))
{
  ABORTIF (state != RESET, 
           "API usage: 'picosat_set_delete' after 'picosat_init'");
  edelete = ndelete;
  setemgr (nmgr);
}

void
picosat_reset_phases (void)
{
  rebias ();
}

void
picosat_reset_scores (void)
{
  Rnk * r;
  hhead = heap + 1;
  for (r = rnks + 1; r <= rnks + max_var; r++)
    {
      CLR (r);
      hpush (r);
    }
}

void
picosat_remove_learned (unsigned percentage)
{
  enter ();
  reset_incremental_usage ();
  reduce (percentage);
  leave ();
}

void
picosat_set_global_default_phase (int phase)
{
  check_ready ();
  ABORTIF (phase < 0, "API usage: 'picosat_set_global_default_phase' "
                      "with negative argument");
  ABORTIF (phase > 3, "API usage: 'picosat_set_global_default_phase' "
                      "with argument > 3");
  defaultphase = phase;
}

void
picosat_set_default_phase_lit (int int_lit, int phase)
{
  unsigned newphase;
  Lit * lit;
  Var * v;

  check_ready ();

  lit = import_lit (int_lit, 1);
  v = LIT2VAR (lit);

  if (phase)
    {
      newphase = (int_lit < 0) == (phase < 0);
      v->defphase = v->phase = newphase;
      v->usedefphase = v->assigned = 1;
    }
  else
    {
      v->usedefphase = v->assigned = 0;
    }
}

void
picosat_set_more_important_lit (int int_lit)
{
  Lit * lit;
  Var * v;
  Rnk * r;

  check_ready ();

  lit = import_lit (int_lit, 1);
  v = LIT2VAR (lit);
  r = VAR2RNK (v);

  ABORTIF (r->lessimportant, "can not mark variable more and less important"); 

  if (r->moreimportant)
    return;

  r->moreimportant = 1;

  if (r->pos)
    hup (r);
}

void
picosat_set_less_important_lit (int int_lit)
{
  Lit * lit;
  Var * v;
  Rnk * r;

  check_ready ();

  lit = import_lit (int_lit, 1);
  v = LIT2VAR (lit);
  r = VAR2RNK (v);

  ABORTIF (r->moreimportant, "can not mark variable more and less important"); 

  if (r->lessimportant)
    return;

  r->lessimportant = 1;

  if (r->pos)
    hup (r);
}

#ifndef NADC

unsigned 
picosat_ado_conflicts (void)
{
  check_ready ();
  return adoconflicts;
}

void
picosat_disable_ado (void)
{
  check_ready ();
  assert (!adodisabled);
  adodisabled = 1;
}

void
picosat_enable_ado (void)
{
  check_ready ();
  assert (adodisabled);
  adodisabled = 0;
}

void
picosat_set_ado_conflict_limit (unsigned newadoconflictlimit)
{
  check_ready ();
  adoconflictlimit = newadoconflictlimit;
}

#endif

void
picosat_simplify (void)
{
  enter ();
  reset_incremental_usage ();
  simplify (1);
  leave ();
}

int
picosat_haveados (void)
{
#ifndef NADC
  return 1;
#else
  return 0;
#endif
}

void
picosat_save_original_clauses (void)
{
  if (saveorig) return;
  ABORTIF (oadded, "API usage: 'picosat_save_original_clauses' too late");
  saveorig = 1;
}

int
picosat_deref_partial (int int_lit) 
{
  check_ready ();
  check_sat_state ();
  ABORTIF (!int_lit, "API usage: can not partial deref zero literal");
  ABORTIF (mtcls, "API usage: deref partial after empty clause generated");
  ABORTIF (!saveorig, "API usage: 'picosat_save_original_clauses' missing");

#ifdef STATS
  derefs++;
#endif

  if (!partial)
    minautarky ();

  return pderef (int_lit);
}
