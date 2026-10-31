#include "aerotelic/colormaps.hpp"

#include <algorithm>
#include <cmath>

namespace aerotelic {

namespace {

using Stops = std::vector<Rgb>;

constexpr double kPi = 3.14159265358979323846;

float Clamp01(double v) { return static_cast<float>(std::min(1.0, std::max(0.0, v))); }

// A map from a formula, sampled at n stops.
Stops Sampled(int n, Rgb (*f)(double)) {
	Stops s;
	s.reserve(n);
	for (int i = 0; i < n; ++i) s.push_back(f(static_cast<double>(i) / (n - 1)));
	return s;
}

// MATLAB's jet: blue, cyan, green, yellow, red.
Rgb Jet(double t) {
	return {Clamp01(std::min(4.0 * t - 1.5, -4.0 * t + 4.5)),
	        Clamp01(std::min(4.0 * t - 0.5, -4.0 * t + 3.5)),
	        Clamp01(std::min(4.0 * t + 0.5, -4.0 * t + 2.5))};
}

// Full-saturation hue from blue (240 degrees) round to red (0).
Rgb Rainbow(double t) {
	double h = 240.0 * (1.0 - t) / 60.0; // 0..4 in sextants
	double x = 1.0 - std::fabs(std::fmod(h, 2.0) - 1.0);
	switch (static_cast<int>(std::floor(h)) % 6) {
	case 0: return {1.0f, Clamp01(x), 0.0f};
	case 1: return {Clamp01(x), 1.0f, 0.0f};
	case 2: return {0.0f, 1.0f, Clamp01(x)};
	case 3: return {0.0f, Clamp01(x), 1.0f};
	default: return {0.0f, 0.0f, 1.0f};
	}
}

// Black, red, yellow, white.
Rgb Hot(double t) {
	return {Clamp01(t / 0.375), Clamp01((t - 0.375) / 0.375), Clamp01((t - 0.75) / 0.25)};
}

Rgb Cool(double t) { return {Clamp01(t), Clamp01(1.0 - t), 1.0f}; }

Rgb Grey(double t) { return {Clamp01(t), Clamp01(t), Clamp01(t)}; }

// Green (2011), the default start 0.5, rotations -1.5, hue 1, gamma 1.
Rgb Cubehelix(double t) {
	const double start = 0.5, rot = -1.5, hue = 1.0;
	double angle = 2.0 * kPi * (start / 3.0 + 1.0 + rot * t);
	double amp = hue * t * (1.0 - t) / 2.0;
	double c = std::cos(angle), s = std::sin(angle);
	return {Clamp01(t + amp * (-0.14861 * c + 1.78277 * s)),
	        Clamp01(t + amp * (-0.29227 * c - 0.90649 * s)),
	        Clamp01(t + amp * (1.97294 * c))};
}

Rgb Byte(int r, int g, int b) { return {r / 255.0f, g / 255.0f, b / 255.0f}; }

	// viridis: 33 of matplotlib's 256 entries
	const Stops kViridis = {
		{0.2670f, 0.0049f, 0.3294f}, {0.2770f, 0.0503f, 0.3757f}, {0.2823f, 0.0950f, 0.4173f}, {0.2829f, 0.1359f, 0.4534f},
		{0.2788f, 0.1755f, 0.4834f}, {0.2706f, 0.2141f, 0.5071f}, {0.2590f, 0.2515f, 0.5247f}, {0.2450f, 0.2877f, 0.5373f},
		{0.2297f, 0.3224f, 0.5457f}, {0.2143f, 0.3556f, 0.5512f}, {0.1994f, 0.3876f, 0.5546f}, {0.1856f, 0.4186f, 0.5568f},
		{0.1727f, 0.4488f, 0.5579f}, {0.1607f, 0.4785f, 0.5581f}, {0.1490f, 0.5081f, 0.5573f}, {0.1378f, 0.5375f, 0.5549f},
		{0.1276f, 0.5669f, 0.5506f}, {0.1211f, 0.5927f, 0.5446f}, {0.1201f, 0.6222f, 0.5349f}, {0.1301f, 0.6514f, 0.5216f},
		{0.1539f, 0.6802f, 0.5042f}, {0.1911f, 0.7084f, 0.4823f}, {0.2394f, 0.7356f, 0.4557f}, {0.2965f, 0.7616f, 0.4242f},
		{0.3607f, 0.7860f, 0.3878f}, {0.4310f, 0.8085f, 0.3465f}, {0.5063f, 0.8288f, 0.3004f}, {0.5857f, 0.8467f, 0.2499f},
		{0.6681f, 0.8620f, 0.1963f}, {0.7519f, 0.8750f, 0.1432f}, {0.8353f, 0.8860f, 0.1026f}, {0.9162f, 0.8961f, 0.1007f},
		{0.9932f, 0.9062f, 0.1439f},
	};

	// plasma: 33 of matplotlib's 256 entries
	const Stops kPlasma = {
		{0.0504f, 0.0298f, 0.5280f}, {0.1324f, 0.0223f, 0.5633f}, {0.1934f, 0.0184f, 0.5903f}, {0.2480f, 0.0144f, 0.6129f},
		{0.2999f, 0.0096f, 0.6316f}, {0.3502f, 0.0044f, 0.6463f}, {0.3994f, 0.0009f, 0.6561f}, {0.4477f, 0.0021f, 0.6602f},
		{0.4949f, 0.0120f, 0.6579f}, {0.5406f, 0.0350f, 0.6486f}, {0.5844f, 0.0686f, 0.6328f}, {0.6260f, 0.1033f, 0.6113f},
		{0.6651f, 0.1386f, 0.5856f}, {0.7018f, 0.1740f, 0.5573f}, {0.7360f, 0.2094f, 0.5279f}, {0.7681f, 0.2448f, 0.4985f},
		{0.7982f, 0.2802f, 0.4695f}, {0.8231f, 0.3113f, 0.4448f}, {0.8501f, 0.3470f, 0.4172f}, {0.8754f, 0.3833f, 0.3900f},
		{0.8990f, 0.4204f, 0.3630f}, {0.9207f, 0.4584f, 0.3362f}, {0.9403f, 0.4976f, 0.3092f}, {0.9575f, 0.5383f, 0.2821f},
		{0.9718f, 0.5804f, 0.2549f}, {0.9830f, 0.6241f, 0.2279f}, {0.9907f, 0.6696f, 0.2016f}, {0.9943f, 0.7167f, 0.1772f},
		{0.9935f, 0.7655f, 0.1569f}, {0.9876f, 0.8160f, 0.1444f}, {0.9763f, 0.8680f, 0.1434f}, {0.9593f, 0.9214f, 0.1516f},
		{0.9400f, 0.9752f, 0.1313f},
	};

	// inferno: 33 of matplotlib's 256 entries
	const Stops kInferno = {
		{0.0015f, 0.0005f, 0.0139f}, {0.0140f, 0.0112f, 0.0719f}, {0.0423f, 0.0281f, 0.1411f}, {0.0820f, 0.0433f, 0.2153f},
		{0.1293f, 0.0473f, 0.2908f}, {0.1834f, 0.0403f, 0.3550f}, {0.2383f, 0.0366f, 0.3964f}, {0.2908f, 0.0456f, 0.4186f},
		{0.3415f, 0.0623f, 0.4294f}, {0.3915f, 0.0809f, 0.4331f}, {0.4412f, 0.0993f, 0.4316f}, {0.4910f, 0.1172f, 0.4256f},
		{0.5409f, 0.1347f, 0.4151f}, {0.5907f, 0.1526f, 0.4003f}, {0.6401f, 0.1714f, 0.3811f}, {0.6887f, 0.1922f, 0.3576f},
		{0.7357f, 0.2159f, 0.3302f}, {0.7751f, 0.2397f, 0.3035f}, {0.8173f, 0.2710f, 0.2704f}, {0.8560f, 0.3070f, 0.2351f},
		{0.8903f, 0.3480f, 0.1983f}, {0.9199f, 0.3934f, 0.1601f}, {0.9443f, 0.4428f, 0.1204f}, {0.9634f, 0.4955f, 0.0791f},
		{0.9771f, 0.5508f, 0.0391f}, {0.9853f, 0.6084f, 0.0242f}, {0.9879f, 0.6677f, 0.0583f}, {0.9849f, 0.7284f, 0.1208f},
		{0.9761f, 0.7900f, 0.1960f}, {0.9625f, 0.8515f, 0.2855f}, {0.9487f, 0.9105f, 0.3953f}, {0.9517f, 0.9606f, 0.5242f},
		{0.9884f, 0.9984f, 0.6449f},
	};

	// magma: 33 of matplotlib's 256 entries
	const Stops kMagma = {
		{0.0015f, 0.0005f, 0.0139f}, {0.0137f, 0.0118f, 0.0687f}, {0.0396f, 0.0311f, 0.1335f}, {0.0743f, 0.0520f, 0.2027f},
		{0.1131f, 0.0655f, 0.2768f}, {0.1590f, 0.0684f, 0.3527f}, {0.2117f, 0.0620f, 0.4186f}, {0.2654f, 0.0602f, 0.4618f},
		{0.3167f, 0.0717f, 0.4854f}, {0.3660f, 0.0903f, 0.4980f}, {0.4147f, 0.1104f, 0.5047f}, {0.4635f, 0.1299f, 0.5077f},
		{0.5128f, 0.1482f, 0.5076f}, {0.5629f, 0.1654f, 0.5047f}, {0.6136f, 0.1818f, 0.4985f}, {0.6649f, 0.1981f, 0.4888f},
		{0.7164f, 0.2150f, 0.4753f}, {0.7611f, 0.2312f, 0.4602f}, {0.8109f, 0.2529f, 0.4393f}, {0.8578f, 0.2799f, 0.4155f},
		{0.8996f, 0.3146f, 0.3910f}, {0.9336f, 0.3588f, 0.3705f}, {0.9585f, 0.4113f, 0.3600f}, {0.9751f, 0.4689f, 0.3631f},
		{0.9857f, 0.5281f, 0.3794f}, {0.9922f, 0.5875f, 0.4063f}, {0.9958f, 0.6463f, 0.4414f}, {0.9973f, 0.7046f, 0.4826f},
		{0.9970f, 0.7624f, 0.5288f}, {0.9954f, 0.8199f, 0.5791f}, {0.9928f, 0.8772f, 0.6331f}, {0.9898f, 0.9343f, 0.6902f},
		{0.9871f, 0.9914f, 0.7495f},
	};

	// cividis: 33 of matplotlib's 256 entries
	const Stops kCividis = {
		{0.0000f, 0.1351f, 0.3048f}, {0.0000f, 0.1579f, 0.3575f}, {0.0000f, 0.1788f, 0.4148f}, {0.0179f, 0.1985f, 0.4412f},
		{0.1034f, 0.2204f, 0.4358f}, {0.1543f, 0.2425f, 0.4301f}, {0.1951f, 0.2644f, 0.4259f}, {0.2309f, 0.2861f, 0.4235f},
		{0.2637f, 0.3078f, 0.4228f}, {0.2947f, 0.3295f, 0.4237f}, {0.3242f, 0.3513f, 0.4263f}, {0.3529f, 0.3732f, 0.4302f},
		{0.3808f, 0.3952f, 0.4357f}, {0.4082f, 0.4174f, 0.4426f}, {0.4352f, 0.4398f, 0.4511f}, {0.4616f, 0.4624f, 0.4620f},
		{0.4887f, 0.4853f, 0.4710f}, {0.5142f, 0.5055f, 0.4726f}, {0.5441f, 0.5289f, 0.4719f}, {0.5744f, 0.5527f, 0.4692f},
		{0.6052f, 0.5768f, 0.4644f}, {0.6364f, 0.6013f, 0.4578f}, {0.6680f, 0.6262f, 0.4493f}, {0.7000f, 0.6515f, 0.4386f},
		{0.7324f, 0.6774f, 0.4257f}, {0.7652f, 0.7037f, 0.4106f}, {0.7985f, 0.7306f, 0.3926f}, {0.8322f, 0.7580f, 0.3715f},
		{0.8664f, 0.7860f, 0.3466f}, {0.9012f, 0.8146f, 0.3170f}, {0.9367f, 0.8438f, 0.2809f}, {0.9731f, 0.8736f, 0.2347f},
		{0.9957f, 0.9093f, 0.2178f},
	};

	// twilight: 33 of matplotlib's 510 entries
	const Stops kTwilight = {
		{0.8858f, 0.8500f, 0.8880f}, {0.8403f, 0.8426f, 0.8653f}, {0.7619f, 0.8071f, 0.8303f}, {0.6672f, 0.7595f, 0.7986f},
		{0.5778f, 0.7061f, 0.7783f}, {0.5029f, 0.6490f, 0.7652f}, {0.4480f, 0.5924f, 0.7557f}, {0.4073f, 0.5287f, 0.7452f},
		{0.3841f, 0.4614f, 0.7309f}, {0.3739f, 0.3905f, 0.7094f}, {0.3699f, 0.3164f, 0.6770f}, {0.3646f, 0.2401f, 0.6298f},
		{0.3506f, 0.1660f, 0.5615f}, {0.3195f, 0.1067f, 0.4671f}, {0.2701f, 0.0755f, 0.3606f}, {0.2200f, 0.0659f, 0.2709f},
		{0.1874f, 0.0771f, 0.2162f}, {0.2258f, 0.0674f, 0.2273f}, {0.2913f, 0.0749f, 0.2576f}, {0.3712f, 0.0917f, 0.2895f},
		{0.4538f, 0.1162f, 0.3097f}, {0.5305f, 0.1544f, 0.3148f}, {0.5966f, 0.2072f, 0.3126f}, {0.6521f, 0.2697f, 0.3129f},
		{0.6981f, 0.3383f, 0.3221f}, {0.7348f, 0.4112f, 0.3450f}, {0.7626f, 0.4872f, 0.3868f}, {0.7817f, 0.5598f, 0.4462f},
		{0.7992f, 0.6363f, 0.5313f}, {0.8199f, 0.7095f, 0.6330f}, {0.8471f, 0.7761f, 0.7401f}, {0.8722f, 0.8285f, 0.8330f},
		{0.8857f, 0.8500f, 0.8857f},
	};

	// turbo: 33 of matplotlib's 256 entries
	const Stops kTurbo = {
		{0.1900f, 0.0718f, 0.2322f}, {0.2250f, 0.1635f, 0.4510f}, {0.2511f, 0.2524f, 0.6337f}, {0.2682f, 0.3382f, 0.7805f},
		{0.2763f, 0.4212f, 0.8912f}, {0.2754f, 0.5011f, 0.9659f}, {0.2586f, 0.5796f, 0.9988f}, {0.2138f, 0.6589f, 0.9796f},
		{0.1584f, 0.7355f, 0.9231f}, {0.1117f, 0.8057f, 0.8452f}, {0.0927f, 0.8655f, 0.7623f}, {0.1201f, 0.9119f, 0.6866f},
		{0.1966f, 0.9490f, 0.5947f}, {0.3051f, 0.9770f, 0.4899f}, {0.4278f, 0.9942f, 0.3857f}, {0.5466f, 0.9991f, 0.2958f},
		{0.6436f, 0.9900f, 0.2336f}, {0.7158f, 0.9688f, 0.2082f}, {0.7952f, 0.9303f, 0.2039f}, {0.8671f, 0.8797f, 0.2139f},
		{0.9267f, 0.8203f, 0.2257f}, {0.9693f, 0.7552f, 0.2266f}, {0.9916f, 0.6841f, 0.2071f}, {0.9964f, 0.5985f, 0.1738f},
		{0.9859f, 0.5048f, 0.1337f}, {0.9619f, 0.4109f, 0.0931f}, {0.9262f, 0.3247f, 0.0584f}, {0.8807f, 0.2533f, 0.0352f},
		{0.8240f, 0.1918f, 0.0197f}, {0.7556f, 0.1373f, 0.0094f}, {0.6754f, 0.0898f, 0.0045f}, {0.5834f, 0.0493f, 0.0049f},
		{0.4796f, 0.0158f, 0.0106f},
	};

// Moreland's cool-to-warm diverging map, the 33-entry table.
const Stops kCoolWarm = {
	Byte(59, 76, 192), Byte(68, 90, 204), Byte(77, 104, 215), Byte(87, 117, 225), Byte(98, 130, 234),
	Byte(108, 142, 241), Byte(119, 154, 247), Byte(130, 165, 251), Byte(141, 176, 254), Byte(152, 185, 255),
	Byte(163, 194, 255), Byte(174, 201, 253), Byte(184, 208, 249), Byte(194, 213, 244), Byte(204, 217, 238),
	Byte(213, 219, 230), Byte(221, 221, 221), Byte(229, 216, 209), Byte(236, 211, 197), Byte(241, 204, 185),
	Byte(245, 196, 173), Byte(247, 187, 160), Byte(247, 177, 148), Byte(247, 166, 135), Byte(244, 154, 123),
	Byte(241, 141, 111), Byte(236, 127, 99), Byte(229, 112, 88), Byte(222, 96, 77), Byte(213, 80, 66),
	Byte(203, 62, 56), Byte(192, 40, 47), Byte(180, 4, 38),
};

// ColorBrewer, 11-class diverging (reversed so low is blue where the map has one).
const Stops kSpectral = {
	Byte(0x5e, 0x4f, 0xa2), Byte(0x32, 0x88, 0xbd), Byte(0x66, 0xc2, 0xa5), Byte(0xab, 0xdd, 0xa4), Byte(0xe6, 0xf5, 0x98),
	Byte(0xff, 0xff, 0xbf), Byte(0xfe, 0xe0, 0x8b), Byte(0xfd, 0xae, 0x61), Byte(0xf4, 0x6d, 0x43), Byte(0xd5, 0x3e, 0x4f),
	Byte(0x9e, 0x01, 0x42),
};
const Stops kRedBlue = {
	Byte(0x05, 0x30, 0x61), Byte(0x21, 0x66, 0xac), Byte(0x43, 0x93, 0xc3), Byte(0x92, 0xc5, 0xde), Byte(0xd1, 0xe5, 0xf0),
	Byte(0xf7, 0xf7, 0xf7), Byte(0xfd, 0xdb, 0xc7), Byte(0xf4, 0xa5, 0x82), Byte(0xd6, 0x60, 0x4d), Byte(0xb2, 0x18, 0x2b),
	Byte(0x67, 0x00, 0x1f),
};
const Stops kRedYellowBlue = {
	Byte(0x31, 0x36, 0x95), Byte(0x45, 0x75, 0xb4), Byte(0x74, 0xad, 0xd1), Byte(0xab, 0xd9, 0xe9), Byte(0xe0, 0xf3, 0xf8),
	Byte(0xff, 0xff, 0xbf), Byte(0xfe, 0xe0, 0x90), Byte(0xfd, 0xae, 0x61), Byte(0xf4, 0x6d, 0x43), Byte(0xd7, 0x30, 0x27),
	Byte(0xa5, 0x00, 0x26),
};
// ColorBrewer, 9-class sequential.
const Stops kBlues = {
	Byte(0xf7, 0xfb, 0xff), Byte(0xde, 0xeb, 0xf7), Byte(0xc6, 0xdb, 0xef), Byte(0x9e, 0xca, 0xe1), Byte(0x6b, 0xae, 0xd6),
	Byte(0x42, 0x92, 0xc6), Byte(0x21, 0x71, 0xb5), Byte(0x08, 0x51, 0x9c), Byte(0x08, 0x30, 0x6b),
};
const Stops kYellowOrangeRed = {
	Byte(0xff, 0xff, 0xcc), Byte(0xff, 0xed, 0xa0), Byte(0xfe, 0xd9, 0x76), Byte(0xfe, 0xb2, 0x4c), Byte(0xfd, 0x8d, 0x3c),
	Byte(0xfc, 0x4e, 0x2a), Byte(0xe3, 0x1a, 0x1c), Byte(0xbd, 0x00, 0x26), Byte(0x80, 0x00, 0x26),
};

std::vector<Colormap> Build() {
	std::vector<Colormap> maps;
	auto add = [&](const char* name, Stops stops, bool diverging = false) {
		maps.push_back(Colormap{name, std::move(stops), diverging});
	};
	add("Turbo", kTurbo);
	add("Jet", Sampled(33, Jet));
	add("Rainbow", Sampled(33, Rainbow));
	add("Viridis", kViridis);
	add("Plasma", kPlasma);
	add("Inferno", kInferno);
	add("Magma", kMagma);
	add("Cividis", kCividis);
	add("Cool-Warm", kCoolWarm, true);
	add("Spectral", kSpectral, true);
	add("Red-Blue", kRedBlue, true);
	add("Red-Yellow-Blue", kRedYellowBlue, true);
	add("Blues", kBlues);
	add("Yellow-Orange-Red", kYellowOrangeRed);
	add("Hot", Sampled(33, Hot));
	add("Cool", Sampled(33, Cool));
	add("Grey", Sampled(33, Grey));
	add("Cubehelix", Sampled(33, Cubehelix));
	add("Twilight", kTwilight);
	return maps;
}

} // namespace

Rgb Colormap::Sample(double t) const {
	if (stops.empty()) return {};
	if (stops.size() == 1) return stops[0];
	if (!(t > 0.0)) return stops.front(); // NaN lands here too
	if (t >= 1.0) return stops.back();
	const double x = t * (stops.size() - 1);
	const size_t i = static_cast<size_t>(x);
	const float f = static_cast<float>(x - i);
	const Rgb& a = stops[i];
	const Rgb& b = stops[std::min(i + 1, stops.size() - 1)];
	return {a.r + (b.r - a.r) * f, a.g + (b.g - a.g) * f, a.b + (b.b - a.b) * f};
}

const std::vector<Colormap>& Colormaps() {
	static const std::vector<Colormap> maps = Build();
	return maps;
}

const Colormap* FindColormap(const std::string& name) {
	for (const Colormap& m : Colormaps())
		if (m.name == name) return &m;
	return nullptr;
}

const Colormap& DefaultColormap() { return Colormaps().front(); }

} // namespace aerotelic
