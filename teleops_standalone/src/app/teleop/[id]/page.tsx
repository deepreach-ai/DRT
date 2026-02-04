import { auth } from "@clerk/nextjs/server";
import { getRobot } from "@/app/actions";
import { redirect } from "next/navigation";
import ControlRoom from "./control-room";

export const dynamic = 'force-dynamic';

export default async function TeleopPage({ params }: { params: Promise<{ id: string }> }) {
  const { userId } = await auth();
  
  // Resolve params
  const { id } = await params;

  if (!userId) {
    redirect("/");
  }

  const robot = await getRobot(id);

  if (!robot) {
    return <div>Robot not found</div>;
  }

  return (
    <div className="h-screen w-screen bg-gray-900 text-white overflow-hidden">
      <ControlRoom robot={robot} userId={userId} />
    </div>
  );
}
