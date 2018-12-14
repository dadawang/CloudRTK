using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

using System.Runtime.InteropServices;

namespace rnx2rtkp_dll
{
    static class PositioningMode
    {
        public const int PMODE_SINGLE = 0;
        public const int PMODE_DGPS = 1;
        public const int PMODE_KINEMA = 2;
        public const int PMODE_STATIC = 3;
        public const int PMODE_MOVEB = 4;
        public const int PMODE_FIXED = 5;
    }

    static class Constants
    {
        public const int MAXERRMSG = 4096;
        public const int MAXSTRPATH = 1024;
        public const int NFREQ = 3;
        public const int NEXOBS = 0;
        public const int MAXANT = 64;
        public const double D2R = Math.PI / 180.0;

        public const int MAXPRNGPS = 32;
        public const int MINPRNGPS = 1;
        public const int NSATGPS = MAXPRNGPS - MINPRNGPS + 1;
#if ENAGLO
        public const int MINPRNGLO = 1;
        public const int MAXPRNGLO = 27;
        public const int NSATGLO = MAXPRNGLO - MINPRNGLO + 1;
#else
        public const int MINPRNGLO = 0;
        public const int MAXPRNGLO = 0;
        public const int NSATGLO = 0;
#endif

#if ENAGAL
        public const int MINPRNGAL = 1;
        public const int MAXPRNGAL = 36;
        public const int NSATGAL = MAXPRNGAL - MINPRNGAL + 1;
#else
        public const int MINPRNGAL = 0;
        public const int MAXPRNGAL = 0;
        public const int NSATGAL = 0;
#endif

#if ENAQZS
        public const int MINPRNQZS = 193;
        public const int MAXPRNQZS = 202;
        public const int NSATQZS = MAXPRNQZS - MINPRNQZS + 1;
#else
        public const int MINPRNQZS = 0;
        public const int MAXPRNQZS = 0;
        public const int NSATQZS = 0;
#endif

#if ENACMP
        public const int MINPRNCMP = 1;
        public const int MAXPRNCMP = 35;
        public const int NSATCMP = MAXPRNCMP - MINPRNCMP + 1;
#else
        public const int MINPRNCMP = 0;
        public const int MAXPRNCMP = 0;
        public const int NSATCMP = 0;
#endif

#if ENAIRN
        public const int MINPRNIRN = 1;
        public const int MAXPRNIRN = 10;
        public const int NSATIRN = MAXPRNIRN - MINPRNIRN + 1;
#else
        public const int MINPRNIRN = 0;
        public const int MAXPRNIRN = 0;
        public const int NSATIRN = 0;
#endif

#if ENALEO
        public const int MINPRNLEO = 1;
        public const int MAXPRNLEO = 10;
        public const int NSATLEO = MAXPRNLEO - MINPRNLEO + 1;
#else
        public const int MINPRNLEO = 0;
        public const int MAXPRNLEO = 0;
        public const int NSATLEO = 0;
#endif

        public const int MINPRNSBS = 120;
        public const int MAXPRNSBS = 120;
        public const int NSATSBS = MAXPRNSBS - MINPRNSBS + 1;


        public const int MAXSAT = NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATSBS + NSATLEO;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct gtime_t
    {
        public Int64 int_time;
        public double sec;

        public gtime_t(Int64 t, double s)
        {
            int_time = t; sec = s;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SnrMask
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public Int32[] ena;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ * 9)]
        public double[] mask;

        public SnrMask(Int32 a, double b)
        {
            ena = new Int32[2];
            for (int i = 0; i < 2; i++) { ena[i] = a; }

            mask = new double[Constants.NFREQ * 9];
            for (int i = 0; i < Constants.NFREQ * 9; i++) { mask[i] = b; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Pcv
    {
        public int sat;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.MAXANT)]
        public byte[] type;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.MAXANT)]
        public byte[] code;
        public gtime_t ts;
        public gtime_t te;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ * 3)]
        public double[,] off;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ * 19)]
        public double[,] var;

        public static Pcv pcvdeft;
        static Pcv()
        {
            pcvdeft.type = new byte[Constants.MAXANT];
            pcvdeft.code = new byte[Constants.MAXANT];
            pcvdeft.ts = new gtime_t(0, 0.0);
            pcvdeft.te = new gtime_t(0, 0.0);
            pcvdeft.off = new double[Constants.NFREQ, 3];
            pcvdeft.var = new double[Constants.NFREQ, 19];
        }

        public static Pcv pcvdefault
        {
            get { return Pcv.pcvdeft; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ExtErr
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public int[] ena;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * Constants.NFREQ * 2)]
        public double[,] cerr;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * Constants.NFREQ * 2)]
        public double[,] perr;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public double[] gpsglob;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public double[] gloicb;

        public static ExtErr exterrdeft;
        static ExtErr()
        {
            exterrdeft.ena = new int[4];
            exterrdeft.cerr = new double[4, Constants.NFREQ * 2];
            exterrdeft.perr = new double[4, Constants.NFREQ * 2];
            exterrdeft.gpsglob = new double[Constants.NFREQ];
            exterrdeft.gloicb = new double[Constants.NFREQ];
        }

        public static ExtErr exterrdefault
        {
            get { return exterrdeft; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ProcOpt
    {
        public int mode;
        public int soltype;
        public int nf;
        public int navsys;
        public double elmin;
        public SnrMask snrmask;
        public int sateph;
        public int modear;
        public int glomodear;
        public int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
        public int maxout;         /* obs outage count to reset bias */
        public int minlock;        /* min lock count to fix ambiguity */
        public int minfix;         /* min fix count to hold ambiguity */
        public int armaxiter;      /* max iteration to resolve ambiguity */
        public int ionoopt;        /* ionosphere option (IONOOPT_???) */
        public int tropopt;        /* troposphere option (TROPOPT_???) */
        public int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
        public int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
        public int niter;          /* number of filter iteration */
        public int codesmooth;     /* code smoothing window size (0:none) */
        public int intpref;        /* interpolate reference obs (for post mission) */
        public int sbascorr;       /* SBAS correction options */
        public int sbassatsel;     /* SBAS satellite selection (0:all) */
        public int rovpos;         /* rover position for fixed mode */
        public int refpos;         /* base position for relative mode */
                                   /* (0:pos in prcopt,  1:average of single pos, */
                                   /*  2:read from file, 3:rinex header, 4:rtcm pos) */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public double[] eratio;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)]
        public double[] err;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] std;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] prn;
        public double sclkstab;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public double[] thresar;
        public double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
        public double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
        public double thresslip;   /* slip threshold of geometry-free phase (m) */
        public double maxtdiff;    /* max difference of time (sec) */
        public double maxinno;     /* reject threshold of innovation (m) */
        public double maxgdop;     /* reject threshold of gdop */

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] baseline;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] ru;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] rb;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 2 * Constants.MAXANT)]
        public byte[,] anttype;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * 3)]
        public double[,] antdel;

        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 2)]
        public Pcv[] pcvr;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.MAXSAT)]
        public byte[] exsats;

        public int maxaveep;      /* max averaging epoches */
        public int initrst;       /* initialize by restart */
        public int outsingle;     /* output single by dgps/float/fix/ppp outage */
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 2 * 256)]
        public byte[,] rnxopt;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public int[] posopt;
        public int syncsol;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * 6 * 11)]
        public double[,] odisp;
        public ExtErr exterr;
        public int freqopt;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        public byte[] pppopt;

        private static ProcOpt poptdeft;
        static ProcOpt()
        {
            poptdeft.mode = PositioningMode.PMODE_KINEMA;
            poptdeft.soltype = 0;
            poptdeft.nf = 2;
            poptdeft.navsys = 0x01;
            poptdeft.elmin = 15.0 * Constants.D2R;
            poptdeft.snrmask = new SnrMask(0, 0.0);           /* elmin,snrmask */
            poptdeft.sateph = 0; poptdeft.modear = 1; poptdeft.glomodear = 1; poptdeft.bdsmodear = 1; /* sateph,modear,glomodear,bdsmodear */
            poptdeft.maxout = 5; poptdeft.minlock = 0; poptdeft.minfix = 10; poptdeft.armaxiter = 1;
            /* maxout,minlock,minfix,armaxiter */
            poptdeft.ionoopt = 0; poptdeft.tropopt = 0; poptdeft.dynamics = 0; poptdeft.tidecorr = 0;
            /* estion,esttrop,dynamics,tidecorr */
            poptdeft.niter = 1; poptdeft.codesmooth = 0; poptdeft.intpref = 0; poptdeft.sbascorr = 0; poptdeft.sbassatsel = 0;
            /* niter,codesmooth,intpref,sbascorr,sbassatsel */
            poptdeft.rovpos = 0; poptdeft.refpos = 0;
            /* rovpos,refpos */
            poptdeft.eratio = new double[Constants.NFREQ];
            poptdeft.eratio[0] = 100.0; poptdeft.eratio[1] = 100.0;
            for (int i = 2; i < Constants.NFREQ; i++) { poptdeft.eratio[i] = 0.0; }
            /* eratio[] */
            poptdeft.err = new double[5]; poptdeft.err[0] = 100.0;
            poptdeft.err[1] = 0.003; poptdeft.err[2] = 0.003; poptdeft.err[3] = 0.0; poptdeft.err[4] = 1.0;
            /* err[] */
            poptdeft.std = new double[3]; poptdeft.std[0] = 30.0; poptdeft.std[1] = 0.03; poptdeft.std[2] = 0.03;
            /* std[] */
            poptdeft.prn = new double[6]; poptdeft.prn[0] = 1E-4; poptdeft.prn[1] = 1E-3; poptdeft.prn[2] = 1E-4;
            poptdeft.prn[3] = 1E-1; poptdeft.prn[4] = 1E-2; poptdeft.prn[5] = 0.0;
            /* prn[] */
            poptdeft.sclkstab = 5E-12;                      /* sclkstab */
            poptdeft.thresar = new double[8]; poptdeft.thresar[0] = 3.0; poptdeft.thresar[1] = 0.9999; poptdeft.thresar[2] = 0.25;
            poptdeft.thresar[3] = 0.1; poptdeft.thresar[4] = 0.05;
            for (int i = 5; i < 8; i++) { poptdeft.thresar[i] = 0.0; }
            /* thresar */
            poptdeft.elmaskar = 0.0; poptdeft.elmaskhold = 0.0; poptdeft.thresslip = 0.05; /* elmaskar,elmaskhold,thresslip */
            poptdeft.maxtdiff = 30.0; poptdeft.maxinno = 30.0; poptdeft.maxgdop = 30.0;  /* maxtdif,maxinno,maxgdop */
            poptdeft.baseline = new double[2];
            for (int i = 0; i < 2; i++) { poptdeft.baseline[i] = 0.0; }
            poptdeft.ru = new double[3];
            for (int i = 0; i < 3; i++) { poptdeft.ru[i] = 0.0; }
            poptdeft.rb = new double[3];
            for (int i = 0; i < 3; i++) { poptdeft.rb[i] = 0.0; }
            /* baseline,ru,rb */
            poptdeft.anttype = new byte[2, Constants.MAXANT]; /* anttype */
            poptdeft.antdel = new double[2, 3] { { 0, 0, 0 }, { 0, 0, 0 } };
            poptdeft.pcvr = new Pcv[2]; for (int i = 0; i < 2; i++) { poptdeft.pcvr[i] = Pcv.pcvdefault; }
            poptdeft.exsats = new byte[Constants.MAXSAT];
            /* antdel,pcv,exsats */
            poptdeft.rnxopt = new byte[2, 256];
            poptdeft.posopt = new int[6];
            poptdeft.odisp = new double[2, 6 * 11];
            poptdeft.exterr = ExtErr.exterrdefault;
            poptdeft.pppopt = new byte[256];
        }

        public static ProcOpt poptdefault
        {
            get { return poptdeft; }
        }
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct Sol
    {
        public gtime_t time;       /* time (GPST) */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] rr;       /* position/velocity (m|m/s) */
                                  /* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] qr;       /* position variance/covariance (m^2) */
                                 /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
                                 /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] qv;       /* velocity variance/covariance (m^2/s^2) */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] dtr;      /* receiver clock bias to time systems (s) */
        public byte type; /* type (0:xyz-ecef,1:enu-baseline) */
        public byte stat; /* solution status (SOLQ_???) */
        public byte ns;   /* number of valid satellites */
        public float age;          /* age of differential (s) */
        public float ratio;        /* AR ratio factor for valiation */
        public float thres;        /* AR ratio threshold for valiation */

        public static Sol solobj;
        static Sol()
        {
            solobj.rr = new double[6];
            solobj.qr = new float[6];
            solobj.qv = new float[6];
            solobj.dtr = new double[6];
        }

        public static Sol soldefault
        {
            get { return solobj; }
        }
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct AmbControl
    {
        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 4)]
        public gtime_t[] epoch;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public int[] n;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] LC;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] LCv;

        public int fixcnt;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.MAXSAT)]
        public byte[] flags;

        public static AmbControl ambcobj;
        static AmbControl()
        {
            ambcobj.epoch = new gtime_t[4];
            for (int i = 0; i < 4; i++) { ambcobj.epoch[i] = new gtime_t(0, 0.0); }

            ambcobj.n = new int[4];
            ambcobj.LC = new double[4];
            ambcobj.LCv = new double[4];

            ambcobj.flags = new byte[Constants.MAXSAT];
        }

        public static AmbControl ambcdefault
        {
            get { return ambcobj; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SatStatus
    {
        public byte sys;
        public byte vs;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] azel;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public double[] resp;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public double[] resc;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.NFREQ)]
        public byte[] vsat;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.NFREQ)]
        public byte[] snr;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.NFREQ)]
        public byte[] fix;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.NFREQ)]
        public byte[] slip;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.NFREQ)]
        public byte[] half;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public int[] lockcnt;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public UInt32[] outc;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public UInt32[] slipc;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.NFREQ)]
        public UInt32[] rejc;

        public double gf;
        public double gf2;
        public double mw;
        public double phw;

        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 2 * Constants.NFREQ)]
        public gtime_t[,] pt;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * Constants.NFREQ)]
        public double[,] ph;

        /*Constructor default*/
        public static SatStatus satsobj;
        static SatStatus()
        {
            satsobj.azel = new double[2];
            satsobj.resp = new double[2];
            satsobj.resc = new double[2];

            satsobj.vsat = new byte[Constants.NFREQ];
            satsobj.snr = new byte[Constants.NFREQ];
            satsobj.fix = new byte[Constants.NFREQ];
            satsobj.slip = new byte[Constants.NFREQ];
            satsobj.half = new byte[Constants.NFREQ];

            satsobj.lockcnt = new int[Constants.NFREQ];
            satsobj.outc = new UInt32[Constants.NFREQ];
            satsobj.rejc = new UInt32[Constants.NFREQ];

            satsobj.pt = new gtime_t[2, Constants.NFREQ];
            for (int i = 0; i < 2; i++) { for (int j = 0; j < Constants.NFREQ; j++) { satsobj.pt[i, j] = new gtime_t(0, 0.0); } }

            satsobj.ph = new double[2, Constants.NFREQ];
        }

        public static SatStatus ssatdefault
        {
            get { return satsobj; }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Rtk
    {
        public Sol sol;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] rb;

        public int nx;
        public int na;
        public double tt;
        public unsafe double *x;
        public unsafe double *P;
        public unsafe double *xa;
        public unsafe double *Pa;
        public int nfix;

        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = Constants.MAXANT)]
        public AmbControl[] ambc;

        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = Constants.MAXANT)]
        public SatStatus[] ssat;

        public int neb;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.MAXERRMSG)]
        public byte[] errbuf;

        public ProcOpt opt;

        /* Constructor */
        public static Rtk rtkobj;
        static Rtk()
        {
            rtkobj.sol = Sol.soldefault;
            rtkobj.rb = new double[6];

            rtkobj.ambc = new AmbControl[Constants.MAXANT];
            for (int i = 0; i < Constants.MAXANT; i++)
            {
                rtkobj.ambc[i] = AmbControl.ambcdefault;
            }
            rtkobj.ssat = new SatStatus[Constants.MAXANT];
            for (int i = 0; i < Constants.MAXANT; i++)
            {
                rtkobj.ssat[i] = SatStatus.ssatdefault;
            }

            rtkobj.errbuf = new byte[Constants.MAXERRMSG];
            rtkobj.opt = ProcOpt.poptdefault;
        }

        public static Rtk rtkdefault
        {
            get { return rtkobj; }
        }
    }


    class RtkPost_Rinex
    {
        const string path_dll = "rnx2rtkp.dll";

        [DllImport(path_dll, EntryPoint = "zeros", CallingConvention = CallingConvention.Cdecl)]
        public static extern unsafe double* Zeros(int n, int m);

        [DllImport(path_dll, EntryPoint = "rtkinit", CallingConvention = CallingConvention.Cdecl)]
        public static extern int RtkInit(ref Rtk rtk, ref ProcOpt opt);

        static unsafe void Main(string[] args)
        {
            Rtk rtk = Rtk.rtkdefault;
            ProcOpt opt = ProcOpt.poptdefault;
            //RtkPost_Rinex.rtkinit(ref rtk, ref opt);

            //IntPtr x = Marshal.AllocHGlobal(4);
            double* x = RtkPost_Rinex.Zeros(3, 1);
            x[1] = 2;
            for (int i = 0; i < 3; i++)
            {
                Console.WriteLine(x[i]);
            }

            Console.ReadKey();
        }
    }
}
