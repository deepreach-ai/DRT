import { getRobots } from "@/app/actions";
import RobotForm from "./robot-form";
import RobotList from "./robot-list";
import Link from "next/link";

export const dynamic = 'force-dynamic';

export default async function AdminPage() {
  const robots = await getRobots();

  return (
    <div className="p-8">
      <div className="flex justify-between items-center mb-8">
        <h1 className="text-3xl font-bold">Admin Panel</h1>
        <Link href="/" className="text-blue-500 hover:underline">Back to Home</Link>
      </div>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
        <div>
          <h2 className="text-xl font-semibold mb-4">Add New Robot</h2>
          <RobotForm />
        </div>
        
        <div>
          <h2 className="text-xl font-semibold mb-4">Existing Robots</h2>
          <RobotList robots={robots} />
        </div>
      </div>
    </div>
  );
}
