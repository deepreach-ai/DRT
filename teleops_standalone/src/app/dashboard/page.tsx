import { auth } from "@clerk/nextjs/server";
import { getMyRobots } from "@/app/actions";
import Link from "next/link";
import { redirect } from "next/navigation";

export const dynamic = 'force-dynamic';

export default async function DashboardPage() {
  const { userId } = await auth();

  if (!userId) {
    redirect("/");
  }

  const robots = await getMyRobots(userId);

  return (
    <div className="p-8">
      <div className="flex justify-between items-center mb-8">
        <h1 className="text-3xl font-bold">Operator Dashboard</h1>
        <div className="flex gap-4">
            <Link href="/" className="text-blue-500 hover:underline">Home</Link>
        </div>
      </div>

      <div>
        <h2 className="text-xl font-semibold mb-4">My Assigned Robots</h2>
        {robots.length === 0 ? (
          <p className="text-gray-500">You have no assigned robots.</p>
        ) : (
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            {robots.map((robot) => (
              <div key={robot.id} className="border p-6 rounded-lg shadow hover:shadow-md transition">
                <div className="flex justify-between items-start mb-4">
                  <h3 className="text-xl font-bold">{robot.robotName}</h3>
                  <span className={`px-2 py-1 rounded text-xs ${robot.status === 'online' ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}`}>
                    {robot.status || 'Offline'}
                  </span>
                </div>
                <p className="text-gray-600 mb-4">{robot.description || "No description"}</p>
                <div className="text-sm text-gray-500 mb-6">
                    <p>Type: {robot.robotType}</p>
                    <p>Location: {robot.location || 'Unknown'}</p>
                </div>
                
                <Link 
                  href={`/teleop/${robot.id}`}
                  className="block w-full text-center bg-blue-600 text-white py-2 rounded hover:bg-blue-700 font-semibold"
                >
                  Enter Control Room
                </Link>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
