'use client'

import { deleteRobot, assignOperator } from "@/app/actions";
import { useState } from "react";
import { type InferSelectModel } from 'drizzle-orm';
import { robots } from '@/db/schema';

type Robot = InferSelectModel<typeof robots>;

export default function RobotList({ robots }: { robots: Robot[] }) {
  return (
    <div className="flex flex-col gap-4">
      {robots.length === 0 && <p className="text-gray-500">No robots found.</p>}
      {robots.map((robot) => (
        <RobotItem key={robot.id} robot={robot} />
      ))}
    </div>
  );
}

function RobotItem({ robot }: { robot: Robot }) {
  const [operatorId, setOperatorId] = useState(robot.assignedOperatorClerkId || "");

  return (
    <div className="border p-4 rounded shadow flex flex-col gap-2">
      <div className="flex justify-between">
        <h3 className="font-bold text-lg">{robot.robotName}</h3>
        <span className={`px-2 py-1 rounded text-xs ${robot.status === 'online' ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}`}>
          {robot.status}
        </span>
      </div>
      <div className="text-sm text-gray-600">
        Type: {robot.robotType} | Backend: {robot.backendType}
      </div>
      
      <div className="flex items-center gap-2 mt-2">
        <label className="text-sm">Assigned Operator (Clerk ID):</label>
        <input 
          value={operatorId}
          onChange={(e) => setOperatorId(e.target.value)}
          className="border p-1 rounded text-sm flex-1"
          placeholder="user_..."
        />
        <button 
          onClick={() => assignOperator(robot.id, operatorId)}
          className="bg-gray-200 hover:bg-gray-300 px-2 py-1 rounded text-sm"
        >
          Save
        </button>
      </div>

      <div className="flex justify-end mt-2">
        <button 
          onClick={() => deleteRobot(robot.id)}
          className="text-red-500 text-sm hover:underline"
        >
          Delete Robot
        </button>
      </div>
    </div>
  );
}
