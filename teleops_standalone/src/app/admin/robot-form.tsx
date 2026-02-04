'use client'

import { createRobot } from "@/app/actions";
import { useRef } from "react";

export default function RobotForm() {
  const formRef = useRef<HTMLFormElement>(null);

  return (
    <form 
      ref={formRef}
      action={async (formData) => {
        await createRobot(formData);
        formRef.current?.reset();
      }}
      className="flex flex-col gap-4 p-4 border rounded shadow"
    >
      <div>
        <label className="block text-sm font-medium">Robot Name</label>
        <input name="robotName" required className="border p-2 w-full rounded" />
      </div>
      
      <div>
        <label className="block text-sm font-medium">Robot Type</label>
        <select name="robotType" required className="border p-2 w-full rounded">
          <option value="franka_panda">Franka Panda</option>
          <option value="ur5">UR5</option>
          <option value="so_arm100">SO ARM100</option>
          <option value="custom">Custom</option>
        </select>
      </div>

      <div>
        <label className="block text-sm font-medium">Backend Type</label>
        <select name="backendType" required className="border p-2 w-full rounded">
          <option value="mock">Mock</option>
          <option value="isaac">Isaac Sim</option>
          <option value="physical">Physical</option>
        </select>
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div>
          <label className="block text-sm font-medium">Host</label>
          <input name="host" defaultValue="localhost" required className="border p-2 w-full rounded" />
        </div>
        <div>
          <label className="block text-sm font-medium">Port</label>
          <input name="port" type="number" defaultValue="8000" required className="border p-2 w-full rounded" />
        </div>
      </div>

      <div>
        <label className="block text-sm font-medium">Video URL (MJPEG)</label>
        <input name="videoUrl" defaultValue="/api/video" className="border p-2 w-full rounded" />
      </div>

      <button type="submit" className="bg-blue-600 text-white p-2 rounded hover:bg-blue-700">
        Add Robot
      </button>
    </form>
  );
}
