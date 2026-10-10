#include "AerotelicBranding.h"
#include "Licensing/AerotelicLicence.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace AerotelicBranding
{
	FString Version()
	{
		static FString Cached;
		if (!Cached.IsEmpty()) return Cached;
		for (const FString& Candidate : { FPaths::ProjectDir() / TEXT("Resources/VERSION"), FPaths::ProjectDir() / TEXT("VERSION") })
		{
			FString Text;
			if (FFileHelper::LoadFileToString(Text, *Candidate))
			{
				Text.TrimStartAndEndInline();
				if (!Text.IsEmpty()) { Cached = Text; return Cached; }
			}
		}
		Cached = TEXT("0.9.0-dev");
		return Cached;
	}

	FString Copyright()
	{
		return FString::Printf(TEXT("© 2026 %s. All rights reserved."), Company);
	}

	TArray<FString> Disclaimers()
	{
		return {
			FString::Printf(TEXT("NO WARRANTY. %s is provided “as is”, without warranty of any kind, express or implied, including but not limited to the implied warranties of merchantability, fitness for a particular purpose and non-infringement."), Product),

			TEXT("NOT A CERTIFIED DESIGN OR ANALYSIS TOOL. Geometry, areas, volumes and derived quantities produced by this software are approximations intended for conceptual and preliminary design. They carry no airworthiness standing and must not be used as the sole basis for certification, structural substantiation, flight-critical decisions, or manufacture."),

			TEXT("VERIFICATION REMAINS THE USER’S. The user is responsible for independently checking every result against established engineering practice and applicable regulation before relying on it."),

			FString::Printf(TEXT("USER-SUPPLIED CONTENT. %s allows the user to import images, drawings, airfoil data and other material of their own choosing for use as modelling references. No such material is supplied with this software, and this software does not retrieve any of it over a network. The user represents and warrants that they hold every right, licence and permission necessary to use the material they import, and that importing, tracing, deriving geometry from, storing or distributing it infringes no copyright, trademark, trade secret, contractual restriction or other right of any third party. The user shall indemnify %s, its suppliers and licensors against any claim, demand, loss or expense arising from material the user imports or from geometry derived from it."), Product, Company),

			FString::Printf(TEXT("LIMITATION OF LIABILITY. In no event shall %s, its suppliers or licensors be liable for any direct, indirect, incidental, special, exemplary or consequential damages arising out of the use of or inability to use this software, however caused and on any theory of liability."), Company),

			TEXT("LICENCE. Use of this software is governed by the licence agreement accompanying it. Reproduction, redistribution or reverse engineering except as that agreement permits is prohibited."),

			FString::Printf(TEXT("LICENCE ACTIVATION. When it is installed and each time it starts, this software contacts the %s licence server to verify its licence. It sends the licence key or its serial number, an identifier derived one-way from this computer's operating-system installation, and the software version. No project, geometry, file or other personal data is sent. Without a connection the software continues to run for up to %d days after the licence was last verified."), Company, AerotelicLicence::GraceDays()),

			TEXT("THIRD-PARTY COMPONENTS. This product includes Open CASCADE Technology (LGPL-2.1 with exception), Unreal Engine (Epic Games, Inc., under the Unreal Engine EULA), yaml-cpp and other open-source components, each licensed by its own terms. No third-party images, drawings or photographs are distributed with this software. Aircraft names and designations are the trademarks of their respective owners and are used here for identification only; no affiliation or endorsement is implied."),
		};
	}

	FString DisclaimerText()
	{
		return FString::Join(Disclaimers(), TEXT("\n\n"));
	}

	FString AboutText()
	{
		return FString::Printf(TEXT("%s %s\n%s\n\nDistributed by %s\n%s\n\nStack: C++ · Unreal Engine 5.8 · Open CASCADE Technology\nSelig and custom airfoils, arbitrary spanwise sections, NURBS planform / twist / dihedral.\n\n%s"),
			Product, *Version(), Tagline, Company, *Copyright(), *DisclaimerText());
	}
}
