#include "aerotelic/materials.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace aerotelic {

namespace {

const char* const kMetal = "Metal";
const char* const kComposite = "Composite";
const char* const kWood = "Wood";
const char* const kFoam = "Foam";
const char* const kResin = "SLA resin";
const char* const kFdm = "FDM thermoplastic";

// The conventions every data-sheet preset follows, so each row's source
// can stay short.
const char* const kResinNote = "post-cured ASTM D638 tensile modulus and strength; allowable = tensile yield where the sheet gives one, else strength at break; nu 0.35 (literature, cured acrylates)";
const char* const kAnycubicNote = "data sheet gives ranges, no post-cure, no tensile modulus (store page adds a Young's modulus range): LOWER bound of E and tensile strength, UPPER bound of density; nu 0.35";
const char* const kFdmNote = "as printed, flat (XY/XZ) orientation, dry; allowable = tensile yield where given, else tensile strength; a printed part is weaker across its layers";

// The alloys added 2026-11-07 all come from one public-domain handbook, so
// each row names only its table, product form, basis and the two strengths.
const char* const kMil5Note = "MIL-HDBK-5J (US DoD, 31 Jan 2003; public domain) design values: E, nu and density from the alloy's design-properties table; allowable = the handbook's MINIMUM tensile yield Fty (longitudinal; A-basis where it gives one, else S-basis) for the product form and thickness named -- a design minimum, not a typical value";

} // namespace

const std::vector<Material>& MaterialPresets() {
	// name, density kg/m^3, E Pa, nu, allowable Pa, category, source
	static const std::vector<Material> presets = {
		// Handbook metals: typical room-temperature figures (MMPDS-01 /
		// ASM Handbook vol. 2); the allowable is the tensile yield.
		{"Aluminium 2024-T3", 2780.0, 73.1e9, 0.33, 345.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, typical room-temperature values, T3 sheet; allowable = tensile yield"},
		{"Aluminium 7075-T6", 2810.0, 71.7e9, 0.33, 503.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, typical room-temperature values, T6 sheet; allowable = tensile yield"},
		{"Aluminium 6061-T6", 2700.0, 68.9e9, 0.33, 276.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, typical room-temperature values, T6; allowable = tensile yield"},
		{"Titanium Ti-6Al-4V", 4430.0, 113.8e9, 0.342, 880.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, annealed, typical; allowable = tensile yield"},
		{"Steel 4130", 7850.0, 205.0e9, 0.29, 435.0e6, kMetal,
			"ASM Handbook vol. 1, normalised at 870 C, typical; allowable = tensile yield"},
		{"Steel 17-4PH H900", 7800.0, 196.0e9, 0.27, 1170.0e6, kMetal,
			"ASM Handbook vol. 1 / AK Steel 17-4 PH data sheet, condition H900, typical; allowable = tensile yield"},
		{"Magnesium AZ31B", 1770.0, 45.0e9, 0.35, 200.0e6, kMetal,
			"ASM Handbook vol. 2, AZ31B-H24 sheet, typical; allowable = tensile yield"},

		// The wider alloy tables (2026-11-07): MIL-HDBK-5J, the US DoD's
		// handbook of aerospace metallic design data (its 2003 edition,
		// since re-issued by Battelle as the copyrighted MMPDS). A US
		// Government work, so its figures may be redistributed. Unlike the
		// seven handbook rows above, whose allowable is a TYPICAL yield,
		// these take the handbook's design MINIMUM yield (A-basis = 99%
		// of material exceeds it with 95% confidence, where the handbook
		// gives one; S-basis = the procurement specification's minimum
		// otherwise), in the longitudinal direction, for the product form
		// and thickness range named -- what an airframe stress report
		// would use. 1 ksi = 6.894757 MPa; 1 lb/in^3 = 27679.9 kg/m^3;
		// E is the tension modulus. Where the handbook leaves Poisson's
		// ratio blank (some titanium alloys) a literature value is given
		// and said to be one.
		//
		// Aluminium: Chapter 3.
		{"Aluminium 2014-T6 sheet", 2796.0, 72.4e9, 0.33, 407.0e6, kMetal,
			std::string("Table 3.2.1.0(b1), AMS 4029 sheet 0.040-0.249 in, A-basis: Ftu 67, Fty 59 ksi; E 10.5e3 ksi, nu 0.33, 0.101 lb/in^3; ") + kMil5Note},
		{"Aluminium 2090-T83 sheet (Al-Li)", 2602.0, 79.3e9, 0.34, 483.0e6, kMetal,
			std::string("Table 3.2.6.0(b), AMS 4251 sheet 0.040-0.125 in, S-basis: Ftu 77, Fty 70 ksi (L; 56 ksi at 45 deg, 66 ksi LT -- strongly anisotropic); E 11.5e3 ksi (L), nu 0.34, 0.094 lb/in^3; ") + kMil5Note},
		{"Aluminium 2124-T851 plate", 2768.0, 71.7e9, 0.33, 393.0e6, kMetal,
			std::string("Table 3.2.7.0(b), AMS 4101 plate 1.000-1.500 in, S-basis: Ftu 66, Fty 57 ksi; E 10.4e3 ksi, nu 0.33, 0.100 lb/in^3; ") + kMil5Note},
		{"Aluminium 2219-T87 plate", 2851.0, 72.4e9, 0.33, 345.0e6, kMetal,
			std::string("Table 3.2.8.0(b2), AMS-QQ-A-250/30 plate 0.250-1.000 in, A-basis: Ftu 63, Fty 50 ksi; E 10.5e3 ksi, nu 0.33, 0.103 lb/in^3; ") + kMil5Note},
		{"Aluminium 5052-H34 sheet", 2685.0, 69.6e9, 0.33, 179.0e6, kMetal,
			std::string("Table 3.5.1.0(b1), AMS 4017 sheet and plate 0.009-1.000 in, S-basis: Ftu 34, Fty 26 ksi; E 10.1e3 ksi, nu 0.33, 0.097 lb/in^3; ") + kMil5Note},
		{"Aluminium 5083-H321 plate", 2657.0, 70.3e9, 0.33, 214.0e6, kMetal,
			std::string("Table 3.5.2.0(b), AMS-QQ-A-250/6 plate 0.250-1.500 in, A-basis: Ftu 44, Fty 31 ksi (28 ksi LT); E 10.2e3 ksi, nu 0.33, 0.096 lb/in^3; ") + kMil5Note},
		{"Aluminium 6013-T6 sheet", 2713.0, 68.3e9, 0.33, 324.0e6, kMetal,
			std::string("Table 3.6.1.0(b), AMS 4216 sheet 0.010-0.062 in, S-basis: Ftu 52, Fty 47 ksi; E 9.9e3 ksi, nu 0.33, 0.098 lb/in^3; ") + kMil5Note},
		{"Aluminium 7050-T7451 plate", 2823.0, 71.0e9, 0.33, 441.0e6, kMetal,
			std::string("Table 3.7.4.0(b1), AMS 4050 plate 0.250-1.500 in, A-basis: Ftu 74, Fty 64 ksi; E 10.3e3 ksi, nu 0.33, 0.102 lb/in^3; ") + kMil5Note},
		{"Aluminium 7055-T77511 extrusion", 2851.0, 71.7e9, 0.33, 621.0e6, kMetal,
			std::string("Table 3.7.5.0(e), AMS 4337 extrusion 0.500-1.500 in, A-basis: Ftu 94, Fty 90 ksi (L; 84 ksi LT); E 10.4e3 ksi, nu 0.33, 0.103 lb/in^3; ") + kMil5Note},
		{"Aluminium 7475-T761 sheet", 2796.0, 68.9e9, 0.33, 421.0e6, kMetal,
			std::string("Table 3.7.10.0(b), AMS 4085 sheet 0.063-0.187 in, S-basis: Ftu 71, Fty 61 ksi; E 10.0e3 ksi, nu 0.33, 0.101 lb/in^3; ") + kMil5Note},
		{"Aluminium A356.0-T6 casting", 2685.0, 71.7e9, 0.33, 193.0e6, kMetal,
			std::string("Table 3.9.5.0(b), AMS-A-21180 casting, designated area, strength class 1, S-basis: Ftu 38, Fty 28 ksi; E 10.4e3 ksi, nu 0.33, 0.097 lb/in^3; ") + kMil5Note},
		{"Aluminium A357.0-T6 casting", 2685.0, 71.7e9, 0.33, 241.0e6, kMetal,
			std::string("Table 3.9.6.0(b), AMS-A-21180 casting, designated area, strength class 1, S-basis: Ftu 45, Fty 35 ksi; E 10.4e3 ksi, nu 0.33, 0.097 lb/in^3; ") + kMil5Note},

		// Steels: Chapter 2 (and A-286 of Chapter 6). The low-alloy steels
		// are specified by the ultimate strength they are heat treated to.
		{"Steel 1025 carbon, annealed", 7861.0, 200.0e9, 0.32, 248.0e6, kMetal,
			std::string("Table 2.2.1.0(b), AMS 5046 sheet, strip and plate, annealed, S-basis: Ftu 55, Fty 36 ksi; E 29.0e3 ksi, nu 0.32, 0.284 lb/in^3; ") + kMil5Note},
		{"Steel 4130, 125 ksi heat treatment", 7833.0, 200.0e9, 0.32, 689.0e6, kMetal,
			std::string("Table 2.3.1.0(c2), AMS 6361 tubing quenched and tempered to 125 ksi, S-basis: Ftu 125, Fty 100 ksi; E 29.0e3 ksi, nu 0.32, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel 4140, 180 ksi heat treatment", 7833.0, 200.0e9, 0.32, 1124.0e6, kMetal,
			std::string("Table 2.3.1.0(g2) at the 180 ksi strength level (Table 2.3.0.2 lists 4140 to 1.00 in for it), quenched and tempered: Ftu 180, Fty 163 ksi; E 29.0e3 ksi, nu 0.32, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel 4340, 200 ksi heat treatment", 7833.0, 200.0e9, 0.32, 1213.0e6, kMetal,
			std::string("Table 2.3.1.0(g2) at the 200 ksi strength level (Table 2.3.0.2 lists 4340 to 1.70-3.50 in for it), quenched and tempered: Ftu 200, Fty 176 ksi; E 29.0e3 ksi, nu 0.32, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel 300M, 280 ksi", 7833.0, 200.0e9, 0.32, 1586.0e6, kMetal,
			std::string("Table 2.3.1.0(f1), AMS 6419 (0.42C 300M) bar, forging and tubing, quenched and tempered, S-basis: Ftu 280, Fty 230 ksi; E 29.0e3 ksi, nu 0.32, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel D6AC, 220 ksi", 7833.0, 200.0e9, 0.32, 1310.0e6, kMetal,
			std::string("Table 2.3.1.0(f1), AMS 6431 bar, forging and tubing, quenched and tempered, S-basis: Ftu 220, Fty 190 ksi; E 29.0e3 ksi, nu 0.32, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel 4330V, 220 ksi", 7833.0, 200.0e9, 0.32, 1276.0e6, kMetal,
			std::string("Table 2.3.1.0(f1), AMS 6411 bar, forging and tubing, quenched and tempered, S-basis: Ftu 220, Fty 185 ksi; E 29.0e3 ksi, nu 0.32, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel H-11 (5Cr-Mo-V), 260 ksi", 7778.0, 206.8e9, 0.36, 1482.0e6, kMetal,
			std::string("Table 2.4.1.0(c), AMS 6487/6488 bar and forging, quenched and tempered, S-basis, transverse column: Ftu 260, Fty 215 ksi; E 30.0e3 ksi, nu 0.36, 0.281 lb/in^3; ") + kMil5Note},
		{"Steel maraging 250", 7916.0, 182.7e9, 0.31, 1724.0e6, kMetal,
			std::string("Table 2.5.1.0(b), AMS 6512 bar under 4.000 in, maraged at 900 F, S-basis: Ftu 255, Fty 250 ksi; E 26.5e3 ksi, nu 0.31, 0.286 lb/in^3; ") + kMil5Note},
		{"Steel AerMet 100", 7889.0, 193.1e9, 0.305, 1620.0e6, kMetal,
			std::string("Table 2.5.3.0(b), AMS 6532 bar and forging, solution treated and aged, A-basis: Ftu 275, Fty 235 ksi; E 28.0e3 ksi, nu 0.305, 0.285 lb/in^3; ") + kMil5Note},
		{"Steel PH13-8Mo H1000", 7723.0, 195.1e9, 0.28, 1310.0e6, kMetal,
			std::string("Table 2.6.6.0(b), AMS 5629 bar, condition H1000: Ftu 201 ksi (A-basis), Fty 190 ksi (S-basis); E 28.3e3 ksi, nu 0.28, 0.279 lb/in^3; ") + kMil5Note},
		{"Steel 15-5PH H1025", 7833.0, 196.5e9, 0.27, 1000.0e6, kMetal,
			std::string("Table 2.6.7.0(b), AMS 5659 bar to 12 in, condition H1025, S-basis: Ftu 155, Fty 145 ksi; E 28.5e3 ksi, nu 0.27, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel 17-4PH H1025", 7833.0, 196.5e9, 0.27, 1000.0e6, kMetal,
			std::string("Table 2.6.9.0(d), AMS 5643 forging, tubing and rings under 8 in, condition H1025, S-basis: Ftu 155, Fty 145 ksi; E 28.5e3 ksi, nu 0.27, 0.283 lb/in^3; ") + kMil5Note},
		{"Steel 17-7PH TH1050", 7640.0, 200.0e9, 0.28, 1034.0e6, kMetal,
			std::string("Table 2.6.10.0(b), AMS 5528 sheet 0.015-0.187 in, condition TH1050, A-basis: Ftu 177, Fty 150 ksi; E 29.0e3 ksi, nu 0.28, 0.276 lb/in^3; ") + kMil5Note},
		{"Steel 301 stainless, annealed", 7916.0, 200.0e9, 0.27, 179.0e6, kMetal,
			std::string("Table 2.7.1.0(b), AMS 5901 sheet and strip, annealed, S-basis: Ftu 73, Fty 26 ksi (L; 30 ksi LT); E 29.0e3 ksi, nu 0.27, 0.286 lb/in^3; ") + kMil5Note},
		{"Steel 301 stainless, half hard", 7916.0, 179.3e9, 0.27, 641.0e6, kMetal,
			std::string("Table 2.7.1.0(b), AMS 5518 sheet and strip, 1/2 hard, A-basis: Ftu 141, Fty 93 ksi (L); E 26.0e3 ksi (L; 28.0e3 LT -- cold work makes it anisotropic), nu 0.27, 0.286 lb/in^3; ") + kMil5Note},
		{"Steel A-286", 7944.0, 200.6e9, 0.31, 655.0e6, kMetal,
			std::string("Table 6.2.1.0(b), AMS 5734 bar to 2.499 in, solution treated and aged, S-basis: Ftu 140, Fty 95 ksi; E 29.1e3 ksi, nu 0.31, 0.287 lb/in^3; ") + kMil5Note},

		// Magnesium: Chapter 4. The casting figures are the handbook's
		// "any area" minimums -- what every point of a sand or investment
		// casting is guaranteed to reach -- and so sit well below the
		// separately cast test-bar figures the alloy specifications quote.
		{"Magnesium AZ31B-F extrusion", 1769.0, 44.8e9, 0.35, 145.0e6, kMetal,
			std::string("Table 4.2.1.0(d), ASTM B107 extruded bar, rod and solid shapes to 0.249 in, as fabricated (F), S-basis: Ftu 35, Fty 21 ksi; E 6.5e3 ksi, nu 0.35, 0.0639 lb/in^3; ") + kMil5Note},
		{"Magnesium AZ61A-F extrusion", 1791.0, 43.4e9, 0.31, 165.0e6, kMetal,
			std::string("Table 4.2.2.0(b), AMS 4350 extruded bar, rod and solid shapes 0.250-2.499 in, as fabricated (F), S-basis: Ftu 40, Fty 24 ksi; E 6.3e3 ksi, nu 0.31, 0.0647 lb/in^3; ") + kMil5Note},
		{"Magnesium ZK60A-T5 extrusion", 1824.0, 44.8e9, 0.35, 248.0e6, kMetal,
			std::string("Table 4.2.3.0(c), AMS 4352 extruded rod, bar and solid shapes under 2.000 in^2, T5, S-basis: Ftu 45, Fty 36 ksi; E 6.5e3 ksi, nu 0.35, 0.0659 lb/in^3; ") + kMil5Note},
		{"Magnesium AM100A-T6 casting", 1802.0, 44.8e9, 0.35, 65.5e6, kMetal,
			std::string("Table 4.3.1.0(b), AMS 4455 investment casting, T6, any area, S-basis: Ftu 17, Fty 9.5 ksi; E 6.5e3 ksi, nu 0.35, 0.0651 lb/in^3; ") + kMil5Note},
		{"Magnesium AZ91C-T6 casting", 1805.0, 44.8e9, 0.35, 82.7e6, kMetal,
			std::string("Table 4.3.2.0(b), AMS 4437 sand casting, T6, any area, S-basis: Ftu 17, Fty 12 ksi; E 6.5e3 ksi, nu 0.35, 0.0652 lb/in^3; ") + kMil5Note},
		{"Magnesium AZ91E-T6 casting", 1805.0, 44.8e9, 0.35, 82.7e6, kMetal,
			std::string("Table 4.3.2.0(c), AMS 4446 sand casting, T6, any area, S-basis: Ftu 17, Fty 12 ksi; E 6.5e3 ksi, nu 0.35, 0.0652 lb/in^3; ") + kMil5Note},
		{"Magnesium AZ92A-T6 casting", 1824.0, 44.8e9, 0.35, 93.1e6, kMetal,
			std::string("Table 4.3.3.0(b), AMS 4434 sand casting, T6, any area, S-basis: Ftu 17, Fty 13.5 ksi; E 6.5e3 ksi, nu 0.35, 0.0659 lb/in^3; ") + kMil5Note},
		{"Magnesium EZ33A-T5 casting", 1824.0, 44.8e9, 0.35, 75.8e6, kMetal,
			std::string("Table 4.3.4.0(b), AMS 4442 sand casting, T5, any area, S-basis: Ftu 13, Fty 11 ksi; E 6.5e3 ksi, nu 0.35, 0.0659 lb/in^3; ") + kMil5Note},
		{"Magnesium QE22A-T6 casting", 1807.0, 44.8e9, 0.35, 159.0e6, kMetal,
			std::string("Table 4.3.5.0(b), AMS 4418 sand casting, T6, any area, S-basis: Ftu 32, Fty 23 ksi; E 6.5e3 ksi, nu 0.35, 0.0653 lb/in^3; ") + kMil5Note},
		{"Magnesium ZE41A-T5 casting", 1816.0, 44.8e9, 0.35, 121.0e6, kMetal,
			std::string("Table 4.3.6.0(b), AMS 4439 sand casting, T5, any area, S-basis: Ftu 26, Fty 17.5 ksi; E 6.5e3 ksi, nu 0.35, 0.0656 lb/in^3; ") + kMil5Note},

		// Titanium: Chapter 5. The handbook names the commercially pure
		// grades CP-1 to CP-4 by falling strength; CP-3 (AMS 4902) is
		// ASTM grade 2 and CP-1 (AMS 4901) is ASTM grade 4.
		{"Titanium CP grade 2 (CP-3)", 4512.0, 106.9e9, 0.34, 276.0e6, kMetal,
			std::string("Table 5.2.1.0(b), AMS 4902 (CP-3) sheet, strip and plate to 1.000 in, annealed, S-basis: Ftu 50, Fty 40 ksi; E 15.5e3 ksi, 0.163 lb/in^3; the handbook gives no nu for CP titanium, 0.34 is the literature value; ") + kMil5Note},
		{"Titanium CP grade 4 (CP-1)", 4512.0, 106.9e9, 0.34, 483.0e6, kMetal,
			std::string("Table 5.2.1.0(b), AMS 4901 (CP-1) sheet, strip and plate to 1.000 in, annealed, S-basis: Ftu 80, Fty 70 ksi; E 15.5e3 ksi, 0.163 lb/in^3; the handbook gives no nu for CP titanium, 0.34 is the literature value; ") + kMil5Note},
		{"Titanium Ti-5Al-2.5Sn", 4484.0, 106.9e9, 0.31, 758.0e6, kMetal,
			std::string("Table 5.3.1.0(b), AMS 4910 sheet 0.015-0.079 in, annealed, A-basis: Ftu 120, Fty 110 ksi; E 15.5e3 ksi, 0.162 lb/in^3; the handbook gives no nu, 0.31 is the literature value for the alpha alloy; ") + kMil5Note},
		{"Titanium Ti-8Al-1Mo-1V", 4373.0, 120.7e9, 0.32, 931.0e6, kMetal,
			std::string("Table 5.3.2.0(b1), AMS 4915 sheet under 0.1875 in, single annealed, S-basis: Ftu 145, Fty 135 ksi; E 17.5e3 ksi, nu 0.32, 0.158 lb/in^3; ") + kMil5Note},
		{"Titanium Ti-6Al-2Sn-4Zr-2Mo", 4540.0, 113.8e9, 0.32, 862.0e6, kMetal,
			std::string("Table 5.3.3.0(b), AMS 4919 sheet to 0.046 in, duplex annealed, A-basis: Ftu 135, Fty 125 ksi; E 16.5e3 ksi, nu 0.32, 0.164 lb/in^3; ") + kMil5Note},
		{"Titanium Ti-6Al-4V STA", 4429.0, 110.3e9, 0.31, 1000.0e6, kMetal,
			std::string("Table 5.4.1.0(b), AMS-T-9046 sheet under 0.1875 in, solution treated and aged, S-basis: Ftu 160, Fty 145 ksi (the annealed sheet's A-basis is Ftu 134, Fty 126 ksi); E 16.0e3 ksi, nu 0.31, 0.160 lb/in^3; ") + kMil5Note},
		{"Titanium Ti-6Al-6V-2Sn", 4540.0, 110.3e9, 0.31, 1000.0e6, kMetal,
			std::string("Table 5.4.2.0(b), AMS 4918 sheet, strip and plate under 0.1875 in, annealed, A-basis: Ftu 155, Fty 145 ksi; E 16.0e3 ksi, nu 0.31, 0.164 lb/in^3; ") + kMil5Note},
		{"Titanium Ti-4.5Al-3V-2Fe-2Mo (SP-700)", 4540.0, 110.3e9, 0.31, 869.0e6, kMetal,
			std::string("Table 5.4.3.0(b), AMS 4899 sheet 0.025-0.063 in, annealed, A-basis: Ftu 134, Fty 126 ksi; E 16.0e3 ksi, 0.164 lb/in^3; the handbook gives no nu, 0.31 is the literature value for the alpha-beta alloy; ") + kMil5Note},
		{"Titanium Ti-13V-11Cr-3Al", 4816.0, 100.0e9, 0.33, 869.0e6, kMetal,
			std::string("Table 5.5.1.0(b), AMS-T-9046 sheet 0.012-0.049 in, annealed, S-basis: Ftu 132, Fty 126 ksi; E 14.5e3 ksi, 0.174 lb/in^3; the handbook gives no nu, 0.33 is the literature value for the beta alloy; ") + kMil5Note},
		{"Titanium Ti-15V-3Cr-3Sn-3Al STA", 4761.0, 104.8e9, 0.33, 965.0e6, kMetal,
			std::string("Table 5.5.2.0(b), AMS 4914 sheet to 0.125 in, solution treated and aged 1000 F/8 h, S-basis: Ftu 145, Fty 140 ksi; E 15.2e3 ksi (L), 0.172 lb/in^3; the handbook gives no nu, 0.33 is the literature value for the beta alloy; ") + kMil5Note},
		{"Titanium Ti-10V-2Fe-3Al STA", 4650.0, 109.6e9, 0.33, 1103.0e6, kMetal,
			std::string("Table 5.5.3.0(b), AMS 4983 die forging under 1.000 in, solution treated and aged, S-basis: Ftu 180, Fty 160 ksi; E 15.9e3 ksi, 0.168 lb/in^3; the handbook gives no nu, 0.33 is the literature value for the beta alloy; ") + kMil5Note},

		// Laminates: representative quasi-isotropic figures, not any one
		// manufacturer's.
		{"CFRP quasi-isotropic", 1600.0, 55.0e9, 0.30, 500.0e6, kComposite,
			"Representative quasi-isotropic carbon/epoxy laminate (standard-modulus fibre, ~60% fibre volume, [0/±45/90]s), in-plane; allowable = a representative laminate tensile strength"},
		{"GFRP quasi-isotropic", 1900.0, 20.0e9, 0.28, 250.0e6, kComposite,
			"Representative quasi-isotropic E-glass/epoxy laminate (~55% fibre volume), in-plane; allowable = a representative laminate tensile strength"},

		// Woods, along the grain, at 12% moisture: the USDA Wood Handbook's
		// clear-wood figures (Table 5-3 domestic, 5-5 imported), density from
		// its specific gravity (oven-dry weight, volume at 12%: SG x 1120),
		// Poisson's ratio nu_LR from its Table 5-1 where the species is
		// listed. The allowable is the compression strength parallel to the
		// grain -- the lower of the two and what a spar cap buckles at. Balsa
		// comes in grades the hobby trade sells by weight (contest-light,
		// medium, hard); its modulus and strength go roughly as its density
		// (Borrega & Gibson 2015), so the lighter and heavier grades are the
		// Handbook's 180 kg/m^3 figures scaled by density. The two plywoods
		// are the solid wood's figures at two-thirds, the cross ply of a
		// three-ply sheet adding little along the face grain (ANC-18's
		// approach); their nu is an in-plane plywood figure, approximate.
		{"Spruce (along grain)", 450.0, 10.0e9, 0.37, 40.0e6, kWood,
			"USDA Wood Handbook (FPL-GTR-190) Table 5-3, Sitka spruce at 12% moisture, parallel to grain; nu_LR 0.372 (Table 5-1); allowable ~ compression strength parallel to grain"},
		{"Balsa, light (100 kg/m^3)", 100.0, 1.9e9, 0.23, 8.3e6, kWood,
			"USDA Wood Handbook Table 5-5 balsa (SG 0.16, 12% moisture: E 3.4 GPa, compression parallel 14.9 MPa) scaled to a 6 lb/ft^3 contest-light grade in proportion to density after Borrega & Gibson (2015); nu_LR 0.229 (Table 5-1); allowable = compression parallel to grain"},
		{"Balsa, medium (160 kg/m^3)", 160.0, 3.0e9, 0.23, 13.0e6, kWood,
			"USDA Wood Handbook Table 5-5 balsa (SG 0.16, 12% moisture: E 3.4 GPa, compression parallel 14.9 MPa) scaled to a 10 lb/ft^3 medium grade in proportion to density after Borrega & Gibson (2015); nu_LR 0.229 (Table 5-1); allowable = compression parallel to grain"},
		{"Balsa, hard (240 kg/m^3)", 240.0, 4.5e9, 0.23, 20.0e6, kWood,
			"USDA Wood Handbook Table 5-5 balsa (SG 0.16, 12% moisture: E 3.4 GPa, compression parallel 14.9 MPa) scaled to a 15 lb/ft^3 hard grade in proportion to density after Borrega & Gibson (2015); nu_LR 0.229 (Table 5-1); allowable = compression parallel to grain"},
		{"Basswood, American (along grain)", 415.0, 10.1e9, 0.36, 32.6e6, kWood,
			"USDA Wood Handbook Table 5-3, American basswood at 12% moisture (SG 0.37): E 10.1 GPa, MOR 60 MPa; nu_LR 0.364 (Table 5-1); allowable = compression parallel to grain 32.6 MPa"},
		{"Pine, Eastern white (along grain)", 390.0, 8.5e9, 0.33, 33.1e6, kWood,
			"USDA Wood Handbook Table 5-3, Eastern white pine at 12% moisture (SG 0.35): E 8.5 GPa, MOR 59 MPa; nu_LR 0.33 as the western white pine of Table 5-1; allowable = compression parallel to grain 33.1 MPa"},
		{"Yellow-poplar (along grain)", 470.0, 10.9e9, 0.32, 38.2e6, kWood,
			"USDA Wood Handbook Table 5-3, yellow-poplar at 12% moisture (SG 0.42): E 10.9 GPa, MOR 70 MPa; nu_LR 0.318 (Table 5-1); allowable = compression parallel to grain 38.2 MPa"},
		{"Douglas-fir, Coast (along grain)", 540.0, 13.4e9, 0.29, 49.9e6, kWood,
			"USDA Wood Handbook Table 5-3, Douglas-fir (Coast) at 12% moisture (SG 0.48): E 13.4 GPa, MOR 85 MPa; nu_LR 0.292 (Table 5-1); allowable = compression parallel to grain 49.9 MPa"},
		{"Birch plywood, aircraft 3-ply (face grain)", 690.0, 9.3e9, 0.20, 37.5e6, kWood,
			"Yellow birch of USDA Wood Handbook Table 5-3 (12% moisture, SG 0.62: E 13.9 GPa, compression parallel 56.3 MPa) at two-thirds for a three-ply sheet of equal veneers along the face grain (ANC-18 Design of Wood Aircraft Structures); nu 0.20 in-plane, approximate; allowable = two-thirds of the compression parallel to grain"},
		{"Poplar plywood, lite-ply 3-ply (face grain)", 470.0, 7.3e9, 0.20, 25.5e6, kWood,
			"Yellow-poplar of USDA Wood Handbook Table 5-3 (12% moisture, SG 0.42: E 10.9 GPa, compression parallel 38.2 MPa) at two-thirds for a three-ply sheet of equal veneers along the face grain (ANC-18); nu 0.20 in-plane, approximate; allowable = two-thirds of the compression parallel to grain. Hobby lite-ply is often a lighter poplar: weigh a sheet"},
		{"Foam, PMI 71 kg/m^3", 71.0, 0.105e9, 0.30, 1.9e6, kFoam,
			"Evonik ROHACELL 71 IG-F data sheet (nominal 75 kg/m^3 grade): E 105 MPa; allowable between the compressive (1.7 MPa) and tensile (2.8 MPa) strengths"},

		// Photopolymer thermosets of resin (SLA / LCD / DLP) printers: the
		// manufacturers' technical data sheets, POST-CURED figures (ASTM D638
		// tensile modulus and ultimate strength; the allowable is the yield
		// where the sheet gives one, else the strength at break), densities
		// from Formlabs' post-cured density table or the sheet. Poisson's
		// ratio is not on the sheets: 0.35 is the measured figure for cured
		// acrylate resins (Formlabs Clear 0.35, mSLA resins 0.34), 0.30 for
		// the glass-filled one, 0.38 for the soft polypropylene-like one.
		{"SLA resin, standard (Formlabs Grey V4)", 1140.0, 2.8e9, 0.35, 65.0e6, kResin,
			std::string("Formlabs Grey Resin V4 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, clear (Formlabs Clear V4.1)", 1170.0, 2.37e9, 0.35, 53.0e6, kResin,
			std::string("Formlabs Clear Resin V4.1 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, tough ABS-like (Formlabs Tough 2000 V2)", 1090.0, 1.8e9, 0.35, 40.4e6, kResin,
			std::string("Formlabs Tough 2000 Resin V2 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, tough (Siraya Tech Blu)", 1150.0, 1.8e9, 0.35, 51.0e6, kResin,
			std::string("Siraya Tech Blu TDS; ") + kResinNote},
		{"SLA resin, durable PP-like (Formlabs Durable)", 1130.0, 1.26e9, 0.38, 31.8e6, kResin,
			std::string("Formlabs Durable Resin TDS; density from Formlabs' post-cured density table; nu 0.38 for the soft polypropylene-like resin; ") + kResinNote},
		{"SLA resin, high-temp (Formlabs High Temp V2)", 1190.0, 2.8e9, 0.35, 58.3e6, kResin,
			std::string("Formlabs High Temp Resin V2 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, glass-filled (Formlabs Rigid 10K)", 1700.0, 11.0e9, 0.30, 88.0e6, kResin,
			std::string("Formlabs Rigid 10K Resin TDS; nu 0.30 for the glass-filled resin; ") + kResinNote},

		// Anycubic's LCD / DLP resins (2026-11-06). Their data sheets give
		// RANGES, not values, name no post-cure, and list no tensile
		// modulus (their "bending strength" and "bending modulus" rows are
		// labelled the wrong way round); the store pages add a Young's
		// modulus range. Taken conservatively: the LOWER bound of the
		// Young's modulus and of the tensile strength (ASTM D638), the UPPER
		// bound of the density. ABS-Like V2 has no Young's modulus anywhere,
		// so its lower flexural modulus stands in. Poisson's ratio 0.35 as
		// for the other acrylates, 0.38 for the ductile Tough 2.0 and 0.42
		// for Tough Ultra, which at 70-80% elongation and Shore D 71-75 is
		// nearly an elastomer -- treat its linear results with care.
		{"SLA resin, standard (Anycubic Standard)", 1200.0, 1.6e9, 0.35, 40.0e6, kResin,
			std::string("Anycubic Standard Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, standard (Anycubic Standard V2)", 1250.0, 1.2e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic Standard Resin V2 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, standard (Anycubic 16K Standard)", 1100.0, 1.8e9, 0.35, 40.0e6, kResin,
			std::string("Anycubic 16K Standard Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like 2.0)", 1090.0, 1.8e9, 0.35, 34.0e6, kResin,
			std::string("Anycubic ABS-Like Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like V2)", 1200.0, 1.0e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic ABS-Like Resin V2 TDS; no Young's modulus anywhere, so the lower flexural modulus stands in; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like 3.0)", 1090.0, 1.3e9, 0.35, 38.0e6, kResin,
			std::string("Anycubic ABS-Like Resin 3.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like Pro 2)", 1250.0, 1.6e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic ABS-Like Resin Pro 2 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, water-wash ABS-like (Anycubic Water-Wash ABS-Like 3.0)", 1100.0, 1.8e9, 0.35, 36.0e6, kResin,
			std::string("Anycubic Water-Wash ABS-Like Resin 3.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, water-wash (Anycubic Water-Wash 2.0)", 1090.0, 1.1e9, 0.35, 38.0e6, kResin,
			std::string("Anycubic Water-Wash Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, tough (Anycubic Tough 2.0)", 1150.0, 0.9e9, 0.38, 30.0e6, kResin,
			std::string("Anycubic Tough Resin 2.0 TDS and store page; nu 0.38 for the ductile resin; ") + kAnycubicNote},
		{"SLA resin, soft tough (Anycubic Tough Ultra)", 1250.0, 0.35e9, 0.42, 20.0e6, kResin,
			std::string("Anycubic Tough Resin Ultra TDS and store page; nu 0.42 -- at 70-80% elongation and Shore D 71-75 nearly an elastomer, so treat linear results with care; ") + kAnycubicNote},
		{"SLA resin, clear (Anycubic High Clear)", 1120.0, 0.7e9, 0.35, 28.0e6, kResin,
			std::string("Anycubic High Clear Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, fast (Anycubic High Speed 2.0)", 1200.0, 1.3e9, 0.35, 37.0e6, kResin,
			std::string("Anycubic High Speed Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, plant-based (Anycubic Plant-Based 2.0)", 1100.0, 1.3e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic Plant-Based Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, DLP (Anycubic DLP Craftsman)", 1200.0, 1.3e9, 0.35, 36.0e6, kResin,
			std::string("Anycubic DLP Craftsman Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, texture (Anycubic 14K Texture)", 1110.0, 1.3e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic 14K Texture Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, bio (Anycubic Bio)", 1150.0, 1.2e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic Bio Resin TDS and store page; ") + kAnycubicNote},

		// Thermoplastics of fused-deposition (FDM / FFF) printers: the
		// filament makers' technical data sheets, AS PRINTED, in the flat
		// (XY / XZ) orientation, dry. A printed part is weaker across its
		// layers (Z strength is typically 60-85% of XY on these sheets) and
		// its stiffness depends on infill and walls: these are the figures
		// for a solid part loaded along the layers, an upper bound for the
		// isotropic analysis. The allowable is the tensile yield where the
		// sheet gives one, else the tensile strength. Poisson's ratios are
		// the measured ones for printed parts (PLA 0.36, ABS 0.37, PETG 0.38,
		// nylon 0.39, PC 0.37, PEEK 0.40, PEI 0.36, 0.35 for the fibre-filled).
		{"FDM PLA (Prusament)", 1240.0, 2.3e9, 0.36, 51.0e6, kFdm,
			std::string("Prusament PLA TDS; nu 0.36 (literature, printed PLA); ") + kFdmNote},
		{"FDM PETG (Prusament)", 1270.0, 1.5e9, 0.38, 47.0e6, kFdm,
			std::string("Prusament PETG TDS; nu 0.38 (literature, printed PETG); ") + kFdmNote},
		{"FDM ABS (Bambu Lab)", 1050.0, 2.2e9, 0.37, 33.0e6, kFdm,
			std::string("Bambu Lab ABS TDS; nu 0.37 (literature, printed ABS); ") + kFdmNote},
		{"FDM ASA (Prusament)", 1070.0, 1.6e9, 0.37, 42.0e6, kFdm,
			std::string("Prusament ASA TDS; nu 0.37 (as ABS); ") + kFdmNote},
		{"FDM polycarbonate (Prusament PC Blend)", 1220.0, 1.9e9, 0.37, 63.0e6, kFdm,
			std::string("Prusament PC Blend TDS; nu 0.37 (literature, printed PC); ") + kFdmNote},
		{"FDM nylon PA6/66 (Polymaker CoPA)", 1120.0, 2.22e9, 0.39, 66.2e6, kFdm,
			std::string("Polymaker PolyMide CoPA TDS; nu 0.39 (literature, printed nylon); ") + kFdmNote},
		{"FDM nylon-CF (Bambu Lab PAHT-CF)", 1060.0, 3.86e9, 0.35, 92.0e6, kFdm,
			std::string("Bambu Lab PAHT-CF TDS; nu 0.35 (fibre-filled); ") + kFdmNote},
		{"FDM nylon-CF (Markforged Onyx)", 1200.0, 2.4e9, 0.35, 37.0e6, kFdm,
			std::string("Markforged composites data sheet, Onyx; nu 0.35 (fibre-filled); ") + kFdmNote},
		{"FDM PEI (Stratasys ULTEM 9085)", 1340.0, 2.52e9, 0.36, 69.2e6, kFdm,
			std::string("Stratasys ULTEM 9085 resin data sheet, XZ orientation column; nu 0.36 (literature, printed PEI); ") + kFdmNote},
		{"FDM PEEK (3DXTech ThermaX)", 1300.0, 3.72e9, 0.40, 100.0e6, kFdm,
			std::string("3DXTech ThermaX PEEK TDS; nu 0.40 (literature, PEEK); ") + kFdmNote},
	};
	return presets;
}

const Material* FindMaterialPreset(const std::string& name) {
	for (const Material& m : MaterialPresets()) {
		if (m.name == name) return &m;
	}
	return nullptr;
}

bool SameMaterial(const Material& a, const Material& b) {
	auto same = [](double x, double y) { return std::abs(x - y) <= 1e-9 * std::max(std::abs(x), std::abs(y)) + 1e-300; };
	return a.name == b.name && same(a.density, b.density) && same(a.youngs_modulus, b.youngs_modulus)
		&& same(a.poisson_ratio, b.poisson_ratio) && same(a.allowable_stress, b.allowable_stress);
}

bool IsMaterialPreset(const Material& m) {
	const Material* p = FindMaterialPreset(m.name);
	return p != nullptr && SameMaterial(m, *p);
}

std::string MaterialProblem(const Material& m) {
	auto blank = [](const std::string& s) { return s.find_first_not_of(" \t\r\n") == std::string::npos; };
	if (blank(m.name)) return "the material has no name";
	if (!std::isfinite(m.density) || m.density <= 0.0) return "the density must be positive";
	if (!std::isfinite(m.youngs_modulus) || m.youngs_modulus <= 0.0) return "Young's modulus must be positive";
	if (!std::isfinite(m.poisson_ratio) || m.poisson_ratio < 0.0 || m.poisson_ratio >= 0.5) return "Poisson's ratio must lie in [0, 0.5)";
	if (!std::isfinite(m.allowable_stress) || m.allowable_stress < 0.0) return "the allowable stress cannot be negative (0 reports no margin)";
	return std::string();
}

std::string WriteMaterialsYaml(const std::vector<Material>& materials) {
	YAML::Node root;
	YAML::Node list(YAML::NodeType::Sequence);
	for (const Material& m : materials) {
		YAML::Node n;
		n["name"] = m.name;
		n["category"] = m.category;
		n["density"] = m.density;
		n["youngs_modulus"] = m.youngs_modulus;
		n["poisson_ratio"] = m.poisson_ratio;
		n["allowable_stress"] = m.allowable_stress;
		n["source"] = m.source;
		list.push_back(n);
	}
	root["materials"] = list;
	YAML::Emitter out;
	// 12 significant digits: 0.3 reads as 0.3, not 0.29999999999999999, and
	// no material figure carries more.
	out.SetDoublePrecision(12);
	out << root;
	return std::string("# Aerotelic material library (Model > Material Properties). SI: kg/m^3, Pa.\n") + out.c_str() + "\n";
}

std::vector<Material> ReadMaterialsYaml(const std::string& text) {
	std::vector<Material> out;
	if (text.find_first_not_of(" \t\r\n") == std::string::npos) return out;
	YAML::Node root;
	try {
		root = YAML::Load(text);
	} catch (const YAML::Exception& e) {
		throw std::runtime_error(std::string("material library: ") + e.what());
	}
	if (!root.IsMap()) throw std::runtime_error("material library: the file is not a mapping with a 'materials' list");
	const YAML::Node list = root["materials"];
	if (!list) return out;
	if (!list.IsSequence()) throw std::runtime_error("material library: 'materials' is not a list");
	try {
		for (const YAML::Node& n : list) {
			if (!n.IsMap() || !n["name"]) throw std::runtime_error("material library: a material has no name");
			Material m;
			m.name = n["name"].as<std::string>();
			if (n["category"]) m.category = n["category"].as<std::string>();
			if (n["density"]) m.density = n["density"].as<double>();
			if (n["youngs_modulus"]) m.youngs_modulus = n["youngs_modulus"].as<double>();
			if (n["poisson_ratio"]) m.poisson_ratio = n["poisson_ratio"].as<double>();
			if (n["allowable_stress"]) m.allowable_stress = n["allowable_stress"].as<double>();
			if (n["source"]) m.source = n["source"].as<std::string>();
			out.push_back(m);
		}
	} catch (const YAML::Exception& e) {
		throw std::runtime_error(std::string("material library: ") + e.what());
	}
	return out;
}

} // namespace aerotelic
