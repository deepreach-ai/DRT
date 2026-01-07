import { SignedIn, SignedOut, SignInButton, UserButton } from "@clerk/nextjs";

export default function Home() {
  return (
    <div className="grid grid-rows-[20px_1fr_20px] items-center justify-items-center min-h-screen p-8 pb-20 gap-16 sm:p-20 font-[family-name:var(--font-geist-sans)]">
      <main className="flex flex-col gap-8 row-start-2 items-center sm:items-start">
        <h1 className="text-4xl font-bold">TeleOps Standalone</h1>
        
        <SignedOut>
          <div className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">
            <SignInButton />
          </div>
        </SignedOut>
        <SignedIn>
          <div className="flex gap-4 items-center">
            <span>Welcome!</span>
            <UserButton />
          </div>
          <div className="flex gap-4">
            <a href="/dashboard" className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded">
              Go to Dashboard
            </a>
            <a href="/admin" className="bg-purple-500 hover:bg-purple-700 text-white font-bold py-2 px-4 rounded">
              Admin Panel
            </a>
          </div>
        </SignedIn>
      </main>
    </div>
  );
}
